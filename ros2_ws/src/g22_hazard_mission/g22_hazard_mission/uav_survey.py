"""UAV mission: take off, fly a lawnmower route, search locally around anything
abnormal, report the estimated source, then come back and land.

States: takeoff -> survey <-> search -> return -> land -> done

The X3 controller takes velocity commands in the body frame, so this node runs
a P controller on the position from odometry (world frame).

Local search: when a reading goes over a trigger, the UAV keeps flying the lane
until the reading starts to fall (it has passed the closest point to the
source), then flies a 3 x 3 grid around the strongest reading, waits at each point to collect readings, and fits the
sensor model to them (search.estimate_source). If the estimate is far from the
grid centre, the grid is moved there and flown again (at most max_iterations).
The estimate is published on /hazards/detections for the coordinator.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from g22_hazard_interfaces.msg import Detection, SensorReading
from g22_hazard_mission.search import estimate_source, grid, lawnmower


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class UavSurvey(Node):

    def __init__(self):
        super().__init__('uav_survey')
        self.altitude = self.declare_parameter('altitude', 6.0).value
        self.max_speed = self.declare_parameter('max_speed', 3.0).value
        self.tolerance = self.declare_parameter('tolerance', 0.5).value
        # x_min, x_max, y_min, y_max of the survey
        self.area = self.declare_parameter('area', [-13.0, 13.0, -10.0, 10.0]).value
        spacing = self.declare_parameter('lane_spacing', 6.0).value
        # readings at 6 m are much weaker than on the ground, so the UAV has its own triggers
        self.gas_trigger = self.declare_parameter('gas_trigger_ppm', 8.0).value
        self.rad_trigger = self.declare_parameter('radiation_trigger_usvh', 0.4).value
        self.background = {
            'gas': self.declare_parameter('gas_background_ppm', 0.5).value,
            'radiation': self.declare_parameter('radiation_background_usvh', 0.1).value,
        }
        self.search_step = self.declare_parameter('search_step', 3.0).value
        self.dwell = self.declare_parameter('dwell_time', 0.5).value
        self.max_iterations = self.declare_parameter('max_iterations', 3).value
        # a new trigger closer than this to a source already found is ignored
        self.min_separation = self.declare_parameter('min_separation', 8.0).value

        self.route = lawnmower(*self.area, spacing)
        self.waypoint = 0

        self.pose = None
        self.home = None
        self.state = 'takeoff'
        self.found = []        # (type, x, y) of the sources already reported
        self.peak = None       # (type, x, y, value): strongest reading since the trigger
        self.next_id = 1

        # local search
        self.search_type = None
        self.search_points = []
        self.search_index = 0
        self.search_iteration = 0
        self.samples = []      # (x, y, value)
        self.dwell_until = None

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(SensorReading, 'reading', self.on_reading, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, 'enable', 10)
        self.detection_pub = self.create_publisher(Detection, '/hazards/detections', 10)
        self.create_timer(0.1, self.tick)

    # ------------------------------------------------------------ inputs ---

    def on_odom(self, msg):
        self.pose = msg.pose.pose
        if self.home is None:
            self.home = (self.pose.position.x, self.pose.position.y)

    def on_reading(self, msg):
        p = msg.position
        if self.state == 'survey':
            kind = None
            if msg.gas_ppm >= self.gas_trigger:
                kind = 'gas'
            elif msg.radiation_usvh >= self.rad_trigger:
                kind = 'radiation'
            if kind and self.already_found(kind, p.x, p.y):
                kind = None
            if self.peak is not None:
                # keep flying the lane while the reading rises, search once it falls again
                value = msg.gas_ppm if self.peak[0] == 'gas' else msg.radiation_usvh
                if value > self.peak[3]:
                    self.peak = (self.peak[0], p.x, p.y, value)
                elif value < 0.8 * self.peak[3]:
                    kind, x, y, _ = self.peak
                    self.peak = None
                    self.start_search(kind, x, y)
            elif kind:
                value = msg.gas_ppm if kind == 'gas' else msg.radiation_usvh
                self.peak = (kind, p.x, p.y, value)
        elif self.state == 'search' and self.dwell_until is not None:
            value = msg.gas_ppm if self.search_type == 'gas' else msg.radiation_usvh
            self.samples.append((p.x, p.y, value))

    def already_found(self, kind, x, y):
        return any(k == kind and math.hypot(x - fx, y - fy) < self.min_separation
                   for k, fx, fy in self.found)

    # ------------------------------------------------------- local search ---

    def start_search(self, kind, x, y):
        self.get_logger().info(f'{kind} reading over the trigger at ({x:.1f}, {y:.1f}), searching')
        self.state = 'search'
        self.search_type = kind
        self.search_iteration = 1
        self.samples = []
        self.set_grid(x, y)

    def set_grid(self, cx, cy):
        x_min, x_max, y_min, y_max = self.area
        self.search_centre = (cx, cy)
        self.search_points = [(min(max(x, x_min), x_max), min(max(y, y_min), y_max))
                              for x, y in grid(cx, cy, self.search_step)]
        self.search_index = 0
        self.dwell_until = None

    def search_tick(self):
        now = self.get_clock().now()
        x, y = self.search_points[self.search_index]
        arrived = self.fly_to(x, y, self.altitude)
        if self.dwell_until is None:
            if arrived:
                self.dwell_until = now + Duration(seconds=self.dwell)
            return
        if now < self.dwell_until:
            return
        self.dwell_until = None
        self.search_index += 1
        if self.search_index < len(self.search_points):
            return
        self.finish_grid()

    def finish_grid(self):
        kind = self.search_type
        estimate = estimate_source(self.samples, kind, self.background[kind])
        if estimate is None:  # no peak in the fit: keep the strongest reading
            estimate = max(self.samples, key=lambda s: s[2])[:2] if self.samples else self.search_centre
        cx, cy = self.search_centre
        moved = math.hypot(estimate[0] - cx, estimate[1] - cy)
        # the fit is still good a bit outside the grid, so only move it for far sources
        if moved > 1.5 * self.search_step and self.search_iteration < self.max_iterations:
            self.search_iteration += 1
            self.get_logger().info(f'source looks {moved:.1f} m away, moving the grid there')
            self.set_grid(*estimate)
            return
        if self.already_found(kind, *estimate):
            self.get_logger().info(f'{kind} source at ({estimate[0]:.1f}, {estimate[1]:.1f}) '
                                   'was already reported')
        else:
            self.report(kind, *estimate)
        self.state = 'survey'

    def report(self, kind, x, y):
        strongest = max((s[2] for s in self.samples), default=0.0)
        msg = Detection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.id = self.next_id
        msg.hazard_type = kind
        msg.position.x, msg.position.y = float(x), float(y)
        msg.value = float(strongest)
        msg.robot = 'uav1'
        self.detection_pub.publish(msg)
        self.found.append((kind, x, y))
        self.get_logger().info(f'#{self.next_id} {kind} source estimated at ({x:.2f}, {y:.2f}) '
                               f'from {len(self.samples)} readings')
        self.next_id += 1

    # ----------------------------------------------------------- control ---

    def fly_to(self, x, y, z):
        """Send one velocity command towards (x, y, z). True once it is there."""
        p = self.pose.position
        ex, ey, ez = x - p.x, y - p.y, z - p.z
        vx, vy = ex, ey
        speed = math.hypot(vx, vy)
        if speed > self.max_speed:
            vx, vy = vx * self.max_speed / speed, vy * self.max_speed / speed
        yaw = yaw_of(self.pose.orientation)
        cmd = Twist()
        cmd.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
        cmd.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
        cmd.linear.z = max(-1.0, min(1.0, ez))
        cmd.angular.z = max(-1.0, min(1.0, -yaw))  # keep facing +x
        self.cmd_pub.publish(cmd)
        return math.hypot(ex, ey) < self.tolerance and abs(ez) < self.tolerance

    def tick(self):
        if self.pose is None or self.state == 'done':
            return
        if self.state == 'takeoff':
            self.enable_pub.publish(Bool(data=True))
            if self.fly_to(*self.home, self.altitude):
                self.get_logger().info(f'at {self.altitude} m, starting the survey')
                self.state = 'survey'
        elif self.state == 'survey':
            if self.fly_to(*self.route[self.waypoint], self.altitude):
                if self.peak is not None:  # end of the lane while still on a peak
                    kind, x, y, _ = self.peak
                    self.peak = None
                    self.start_search(kind, x, y)
                    return
                self.waypoint += 1
                if self.waypoint == len(self.route):
                    self.get_logger().info(f'survey done, {len(self.found)} sources found, going home')
                    self.state = 'return'
        elif self.state == 'search':
            self.search_tick()
        elif self.state == 'return':
            if self.fly_to(*self.home, self.altitude):
                self.state = 'land'
        elif self.state == 'land':
            if self.fly_to(*self.home, 0.15):
                self.cmd_pub.publish(Twist())
                self.enable_pub.publish(Bool(data=False))
                self.get_logger().info('landed')
                self.state = 'done'


def main():
    rclpy.init()
    node = UavSurvey()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
