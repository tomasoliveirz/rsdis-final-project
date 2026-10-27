"""UAV mission: take off from the pad and fly a lawnmower route over the yard.
When a reading goes above the threshold, search around it for the source and
report it as a hazard candidate, then carry on with the route. At the end fly
back to the take-off point and land.

The X3 controller takes velocity commands in the body frame, so this node runs
a P controller on the position from odometry (world frame).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from g22_hazard_interfaces.msg import Hazard, SensorReading
from g22_hazard_mission.search import CompassSearch, lawnmower


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class UavSurvey(Node):

    def __init__(self):
        super().__init__('uav_survey')
        self.altitude = self.declare_parameter('altitude', 6.0).value
        self.max_speed = self.declare_parameter('max_speed', 2.0).value
        self.tolerance = self.declare_parameter('tolerance', 0.4).value
        # x_min, x_max, y_min, y_max of the survey
        area = self.declare_parameter('area', [-13.0, 13.0, -10.0, 10.0]).value
        spacing = self.declare_parameter('lane_spacing', 6.0).value
        self.gas_threshold = self.declare_parameter('gas_threshold_ppm', 5.0).value
        self.rad_threshold = self.declare_parameter('radiation_threshold_usvh', 0.5).value
        # readings this close to an area already searched are not searched again
        self.min_separation = self.declare_parameter('min_separation', 6.0).value
        self.search_step = self.declare_parameter('search_step', 2.0).value
        self.search_min_step = self.declare_parameter('search_min_step', 0.5).value
        self.n_samples = self.declare_parameter('samples_per_probe', 3).value

        self.route = lawnmower(*area, spacing)
        self.waypoint = 0

        self.pose = None
        self.home = None
        self.state = 'takeoff'
        self.found = []
        self.searched = []  # (type, x, y) of every search, found or not
        self.search = None
        self.search_type = None
        self.samples = None

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, 'enable', 10)
        self.create_subscription(SensorReading, 'reading', self.on_reading, 10)
        # transient local, so nodes started later still get the hazards already found
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.hazard_pub = self.create_publisher(Hazard, '/hazards/candidates', latched)
        self.create_timer(0.1, self.tick)

    def on_odom(self, msg):
        self.pose = msg.pose.pose
        if self.home is None:
            self.home = (self.pose.position.x, self.pose.position.y)

    def value(self, msg, kind):
        return msg.gas_ppm if kind == Hazard.GAS else msg.radiation_usvh

    def threshold(self, kind):
        return self.gas_threshold if kind == Hazard.GAS else self.rad_threshold

    def on_reading(self, msg):
        if self.state == 'survey':
            kind = max((Hazard.GAS, Hazard.RADIATION),
                       key=lambda k: self.value(msg, k) / self.threshold(k))
            p = msg.position
            if self.value(msg, kind) < self.threshold(kind) or self.was_searched(kind, p):
                return
            self.get_logger().info(f'high reading at ({p.x:.1f}, {p.y:.1f}), searching')
            self.search = CompassSearch(p.x, p.y, self.search_step, self.search_min_step)
            self.search_type = kind
            self.state = 'search'
        elif self.state == 'search' and self.samples is not None:
            self.samples.append(self.value(msg, self.search_type))

    def was_searched(self, kind, p):
        return any(k == kind and math.hypot(x - p.x, y - p.y) < self.min_separation
                   for k, x, y in self.searched)

    def finish_search(self):
        kind, s = self.search_type, self.search
        self.searched.append((kind, s.x, s.y))
        if s.best >= self.threshold(kind):
            self.report(kind, s.x, s.y, s.best)
        else:
            self.get_logger().info(f'false alarm at ({s.x:.1f}, {s.y:.1f})')
        self.state = 'survey'

    def report(self, kind, x, y, value):
        h = Hazard()
        h.header.stamp = self.get_clock().now().to_msg()
        h.header.frame_id = 'map'
        h.id = len(self.found) + 1
        h.type = kind
        h.position.x, h.position.y = x, y
        h.value = float(value)
        h.found_by = self.get_namespace().strip('/')
        self.found.append(h)
        self.hazard_pub.publish(h)
        name = 'gas' if kind == Hazard.GAS else 'radiation'
        self.get_logger().info(f'hazard {h.id}: {name} {value:.2f} at ({x:.1f}, {y:.1f})')

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
        if self.pose is None:
            return
        if self.state == 'takeoff':
            self.enable_pub.publish(Bool(data=True))
            if self.fly_to(*self.home, self.altitude):
                self.get_logger().info(f'at {self.altitude} m, starting the survey')
                self.state = 'survey'
        elif self.state == 'survey':
            if self.fly_to(*self.route[self.waypoint], self.altitude):
                self.waypoint += 1
                if self.waypoint == len(self.route):
                    self.get_logger().info(f'survey done, {len(self.found)} hazards found')
                    self.state = 'return'
        elif self.state == 'search':
            if self.fly_to(*self.search.target, self.altitude) and self.samples is None:
                self.samples = []
            if self.samples is not None and len(self.samples) >= self.n_samples:
                self.search.add(sum(self.samples) / len(self.samples))
                self.samples = None
                if self.search.done:
                    self.finish_search()
        elif self.state == 'return':
            if self.fly_to(*self.home, self.altitude):
                self.state = 'land'
        elif self.state == 'land':
            self.fly_to(*self.home, 0.0)
            if self.pose.position.z < 0.15:
                self.cmd_pub.publish(Twist())
                self.enable_pub.publish(Bool(data=False))
                self.get_logger().info('landed')
                self.state = 'landed'


def main():
    rclpy.init()
    node = UavSurvey()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
