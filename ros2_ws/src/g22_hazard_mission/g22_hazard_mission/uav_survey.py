"""Survey, inspect camera signs or sensor alarms, hand candidates to the UGVs.

A sign requests five readings at its position. An alarm without a sign requests
at most nine local probes. Each inspection has a 25 s limit. Neither path reads
the simulated field model, source positions or background values.
"""
import math

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from g22_hazard_mission import run_node

from g22_hazard_interfaces.msg import Detection, SensorReading, Sign
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
        self.inspection_timeout = self.declare_parameter('inspection_timeout', 25.0).value
        self.n_samples = self.declare_parameter('samples_per_probe', 5).value
        self.min_separation = self.declare_parameter('min_separation', 6.0).value
        if self.n_samples < 1 or self.inspection_timeout <= 0 or any(value <= 0 for value in self.triggers.values()):
            raise ValueError('sample count, inspection timeout and alarm thresholds must be positive')

        self.route = lawnmower(*self.area, spacing)
        self.waypoint = 0
        self.pose = None
        self.home = None
        self.state = 'takeoff'
        self.found = []
        self.searched = []
        self.search = None
        self.pending_signs = []
        self.seen_signs = set()
        self.hazard_type = None
        self.target = None
        self.measure_started = None
        self.inspection_until = None
        self.peak = None
        self.samples = None

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(SensorReading, 'reading', self.on_reading, 10)
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Sign, '/signs', self.on_sign, latched)
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

    def on_sign(self, msg):
        if msg.id in self.seen_signs or msg.type not in (Sign.GAS, Sign.RADIATION):
            return
        if not all(math.isfinite(v) for v in (msg.position.x, msg.position.y)):
            return
        self.seen_signs.add(msg.id)
        self.pending_signs.append(msg)
        if self.state == 'inspect' and self.search is not None:
            sign = self.next_sign(self.hazard_type)
            if sign is not None:
                self.inspect_sign(sign)

    def next_sign(self, kind=None):
        self.pending_signs = [s for s in self.pending_signs
                              if not self.near_known_position(
                                  'gas' if s.type == Sign.GAS else 'radiation',
                                  s.position.x, s.position.y, self.found)]
        matching = [s for s in self.pending_signs
                    if kind is None or s.type == (Sign.GAS if kind == 'gas' else Sign.RADIATION)]
        if not matching or self.pose is None:
            return None
        p = self.pose.position
        sign = min(matching, key=lambda s: math.hypot(s.position.x - p.x, s.position.y - p.y))
        self.pending_signs.remove(sign)
        return sign

    def on_reading(self, msg):
        if self.state == 'survey' and not self.pending_signs:
            kind = self.select_alarm(msg)
            if kind is not None:
                self.start_inspection(kind, msg.position.x, msg.position.y)
        elif self.state == 'inspect':
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

    def inspect_sign(self, sign):
        kind = 'gas' if sign.type == Sign.GAS else 'radiation'
        xmin, xmax, ymin, ymax = self.area
        x = min(xmax, max(xmin, sign.position.x))
        y = min(ymax, max(ymin, sign.position.y))
        self.start_inspection(kind, x, y, camera=True)

    def start_inspection(self, kind, x, y, camera=False):
        self.search = None if camera else CompassSearch(x, y, self.search_step, bounds=self.area)
        if not camera:
            self.searched.append((kind, x, y))
        self.target, self.hazard_type = (x, y), kind
        self.samples = self.measure_started = None
        self.peak = None
        self.inspection_until = self.get_clock().now() + Duration(seconds=self.inspection_timeout)
        self.state = 'inspect'
        source = 'camera sign: measure here' if camera else 'sensor alarm: at most nine probes'
        self.get_logger().info(f'{kind} at ({x:.1f}, {y:.1f}): {source}, limit {self.inspection_timeout:.0f} s')

    def collect_sample(self, msg):
        if self.samples is None or not self.at_position(msg.position, *self.target, self.altitude):
            return
        if Time.from_msg(msg.header.stamp).nanoseconds < self.measure_started.nanoseconds:
            return
        value = self.value(msg, self.hazard_type)
        if math.isfinite(value):
            self.samples.append(value)

    def inspection_tick(self):
        if self.get_clock().now() >= self.inspection_until:
            self.finish_inspection('time limit')
            return
        if self.fly_to(*self.target, self.altitude) and self.samples is None:
            self.samples = []
            self.measure_started = self.get_clock().now()
        if self.samples is None or len(self.samples) < self.n_samples:
            return
        value = sum(self.samples) / len(self.samples)
        if self.peak is None or value > self.peak[0]:
            self.peak = (value, *self.target)
        if self.search is None:
            self.finish_inspection('camera position measured')
            return
        self.search.add(value)
        if self.search.done:
            self.finish_inspection('local probes complete')
        else:
            self.target = self.search.target
            self.samples = self.measure_started = None

    def finish_inspection(self, reason):
        kind = self.hazard_type
        if self.peak is not None:
            value, x, y = self.peak
            if value >= self.triggers[kind] and not self.near_known_position(kind, x, y, self.found):
                self.report(kind, x, y, value)
            else:
                self.get_logger().info(f'{kind}: inspection ended below alarm or near a known candidate')
        else:
            self.get_logger().warn(f'{kind}: inspection ended without fresh measurements')
        self.get_logger().info(f'inspection finished: {reason}; resuming survey')
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
        self.get_logger().info(f'#{msg.id} {kind}: measured candidate {value:.2f} at ({x:.2f}, {y:.2f})')

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
                   'inspect': self.inspection_tick, 'return': self.return_home, 'land': self.land}
        action = actions.get(self.state)
        if action is not None:
            action()

    def take_off(self):
        self.enable_pub.publish(Bool(data=True))
        if self.fly_to(*self.home, self.altitude):
            self.get_logger().info(f'at {self.altitude} m, starting the survey')
            self.state = 'survey'

    def follow_route(self):
        sign = self.next_sign()
        if sign is not None:
            self.inspect_sign(sign)
            return
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
