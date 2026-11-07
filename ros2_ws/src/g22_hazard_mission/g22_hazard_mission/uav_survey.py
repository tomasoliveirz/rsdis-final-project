"""Survey the yard, follow high sensor readings, report candidates and return home.

The local search compares averaged readings at neighbouring positions. It does
not use the simulated field model, source positions or background values.
The X3 takes body-frame velocity commands; odometry gives its world pose.
"""
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from g22_hazard_mission import run_node

from g22_hazard_interfaces.msg import Detection, SensorReading
from g22_hazard_mission.search import CompassSearch, inside_bounds, lawnmower


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class UavSurvey(Node):

    def __init__(self):
        super().__init__('uav_survey')
        self.altitude = self.declare_parameter('altitude', 6.0).value
        self.max_speed = self.declare_parameter('max_speed', 3.0).value
        self.tolerance = self.declare_parameter('tolerance', 0.4).value
        self.area = self.declare_parameter('area', [-13.0, 13.0, -10.0, 10.0]).value
        spacing = self.declare_parameter('lane_spacing', 6.0).value
        self.triggers = {
            'gas': self.declare_parameter('gas_trigger_ppm', 8.0).value,
            'radiation': self.declare_parameter('radiation_trigger_usvh', 0.4).value,
        }
        self.search_step = self.declare_parameter('search_step', 2.0).value
        self.search_min_step = self.declare_parameter('search_min_step', 0.5).value
        self.n_samples = self.declare_parameter('samples_per_probe', 10).value
        self.min_separation = self.declare_parameter('min_separation', 6.0).value
        if self.n_samples < 1 or any(value <= 0 for value in self.triggers.values()):
            raise ValueError('sample count and alarm thresholds must be positive')

        self.route = lawnmower(*self.area, spacing)
        self.waypoint = 0
        self.pose = None
        self.home = None
        self.state = 'takeoff'
        self.found = []
        self.searched = []
        self.search = None
        self.search_type = None
        self.samples = None

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(SensorReading, 'reading', self.on_reading, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.enable_pub = self.create_publisher(Bool, 'enable', 10)
        self.detection_pub = self.create_publisher(Detection, '/hazards/detections', 10)
        self.mission_timer = self.create_timer(0.1, self.tick)

    def on_odom(self, msg):
        self.pose = msg.pose.pose
        if self.home is None:
            self.home = (self.pose.position.x, self.pose.position.y)

    @staticmethod
    def value(msg, kind):
        return msg.gas_ppm if kind == 'gas' else msg.radiation_usvh

    def near_known_position(self, kind, x, y, positions):
        return any(k == kind and math.hypot(x - px, y - py) < self.min_separation
                   for k, px, py in positions)

    def on_reading(self, msg):
        if self.state == 'survey':
            kind = self.select_alarm(msg)
            if kind is not None:
                self.start_search(kind, msg.position)
        elif self.state == 'search':
            self.collect_sample(msg)

    def select_alarm(self, msg):
        p = msg.position
        if not inside_bounds(p.x, p.y, self.area):
            return None
        known = self.found + self.searched
        kinds = [kind for kind, trigger in self.triggers.items()
                 if self.value(msg, kind) >= trigger
                 and not self.near_known_position(kind, p.x, p.y, known)]
        return max(kinds, key=lambda k: self.value(msg, k) / self.triggers[k], default=None)

    def start_search(self, kind, position):
        x, y = position.x, position.y
        self.search = CompassSearch(x, y, self.search_step, self.search_min_step, bounds=self.area)
        self.searched.append((kind, x, y))
        self.search_type = kind
        self.samples = None
        self.state = 'search'
        self.get_logger().info(f'{kind} alarm at ({x:.1f}, {y:.1f}), searching')

    def collect_sample(self, msg):
        if self.samples is None or not self.at_position(msg.position, *self.search.target, self.altitude):
            return
        value = self.value(msg, self.search_type)
        if math.isfinite(value):
            self.samples.append(value)

    def search_tick(self):
        if self.fly_to(*self.search.target, self.altitude) and self.samples is None:
            self.samples = []
        if self.samples is None or len(self.samples) < self.n_samples:
            return
        self.search.add(sum(self.samples) / len(self.samples))
        self.samples = None
        if self.search.done:
            self.finish_search()

    def finish_search(self):
        kind, search = self.search_type, self.search
        if search.best >= self.triggers[kind] and not self.near_known_position(
                kind, search.x, search.y, self.found):
            self.report(kind, search.x, search.y, search.best)
        self.state = 'survey'

    def candidate_message(self, kind, x, y, value):
        msg = Detection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.id = len(self.found) + 1
        msg.hazard_type = kind
        msg.position.x, msg.position.y = float(x), float(y)
        msg.value = float(value)
        msg.robot = self.get_namespace().strip('/')
        return msg

    def report(self, kind, x, y, value):
        msg = self.candidate_message(kind, x, y, value)
        self.detection_pub.publish(msg)
        self.found.append((kind, x, y))
        self.get_logger().info(f'#{msg.id} {kind}: measured peak {value:.2f} at ({x:.2f}, {y:.2f})')

    def at_position(self, position, x, y, z):
        return (math.hypot(x - position.x, y - position.y) < self.tolerance
                and abs(z - position.z) < self.tolerance)

    def velocity_command(self, ex, ey, ez):
        vx, vy = ex, ey
        speed = math.hypot(vx, vy)
        if speed > self.max_speed:
            vx, vy = vx * self.max_speed / speed, vy * self.max_speed / speed
        yaw = yaw_of(self.pose.orientation)
        cmd = Twist()
        cmd.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
        cmd.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
        cmd.linear.z = max(-1.0, min(1.0, ez))
        cmd.angular.z = max(-1.0, min(1.0, -yaw))
        return cmd

    def fly_to(self, x, y, z):
        p = self.pose.position
        self.cmd_pub.publish(self.velocity_command(x - p.x, y - p.y, z - p.z))
        return self.at_position(p, x, y, z)

    def tick(self):
        if self.pose is None or self.state == 'done':
            return
        actions = {'takeoff': self.take_off, 'survey': self.follow_route,
                   'search': self.search_tick, 'return': self.return_home, 'land': self.land}
        action = actions.get(self.state)
        if action is not None:
            action()

    def take_off(self):
        self.enable_pub.publish(Bool(data=True))
        if self.fly_to(*self.home, self.altitude):
            self.get_logger().info(f'at {self.altitude} m, starting the survey')
            self.state = 'survey'

    def follow_route(self):
        if self.fly_to(*self.route[self.waypoint], self.altitude):
            self.waypoint += 1
            if self.waypoint == len(self.route):
                self.get_logger().info(f'survey done, {len(self.found)} candidates, going home')
                self.state = 'return'

    def return_home(self):
        if self.fly_to(*self.home, self.altitude):
            self.state = 'land'

    def land(self):
        self.fly_to(*self.home, 0.0)
        if self.pose.position.z < 0.15:
            self.cmd_pub.publish(Twist())
            self.enable_pub.publish(Bool(data=False))
            self.get_logger().info('landed')
            self.state = 'done'
            self.mission_timer.cancel()


def main():
    run_node(UavSurvey)
