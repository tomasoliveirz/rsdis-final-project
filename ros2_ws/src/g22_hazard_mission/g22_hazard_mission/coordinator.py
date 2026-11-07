"""Send the nearest available UGV to each aerial candidate and check it on the ground.

Failed tasks are offered to the other robot. Navigation failures and missing
measurements do not count as negative readings or as confirmations.
"""
import math

import numpy as np
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid

from g22_hazard_mission import run_node

from g22_hazard_interfaces.msg import Detection, SensorReading


class Robot:
    def __init__(self, name):
        self.name = name
        self.position = None   # from its last SensorReading (map frame)
        self.state = 'idle'    # idle, driving, measuring
        self.task = None       # Detection being checked
        self.failed = set()
        self.goal = None
        self.measure_started = None
        self.readings = []     # values collected while measuring
        self.measure_until = None


class Coordinator(Node):

    def __init__(self):
        super().__init__('coordinator')
        names = self.declare_parameter('robots', ['ugv1', 'ugv2']).value
        map_topic = self.declare_parameter('map_topic', '/ugv1/map').value
        self.clearance = self.declare_parameter('goal_clearance', 0.5).value
        self.measure_time = self.declare_parameter('measure_time', 3.0).value
        self.min_samples = self.declare_parameter('min_samples', 3).value
        self.arrival_tolerance = self.declare_parameter('arrival_tolerance', 0.75).value
        self.confirm = {
            'gas': self.declare_parameter('gas_confirm_ppm', 5.0).value,
            'radiation': self.declare_parameter('radiation_confirm_usvh', 0.5).value,
        }

        self.robots = {n: Robot(n) for n in names}
        self.queue = []
        self.map = None

        self.nav = {n: ActionClient(self, NavigateToPose, f'/{n}/navigate_to_pose') for n in names}
        for n in names:
            self.create_subscription(SensorReading, f'/{n}/reading',
                                     lambda msg, n=n: self.on_reading(n, msg), 10)
        self.create_subscription(Detection, '/hazards/detections', self.on_detection, 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, map_topic, self.on_map, map_qos)
        self.pub = self.create_publisher(Detection, '/hazards/confirmations', 10)
        self.create_timer(0.5, self.tick)
        self.get_logger().info(f'coordinating {", ".join(names)}')

    def on_map(self, msg):
        self.map = msg

    def on_reading(self, name, msg):
        robot = self.robots[name]
        robot.position = msg.position
        if robot.state == 'measuring':
            self.collect_ground_reading(robot, msg)

    def collect_ground_reading(self, robot, msg):
        if not self.at_goal(robot):
            return
        if Time.from_msg(msg.header.stamp).nanoseconds < robot.measure_started.nanoseconds:
            return
        value = msg.gas_ppm if robot.task.hazard_type == 'gas' else msg.radiation_usvh
        if math.isfinite(value):
            robot.readings.append(value)

    def on_detection(self, msg):
        if msg.hazard_type not in self.confirm or not all(
                math.isfinite(v) for v in (msg.position.x, msg.position.y)):
            self.get_logger().warn('ignoring an invalid hazard candidate')
            return
        self.get_logger().info(f'#{msg.id} {msg.hazard_type} reported by {msg.robot} at '
                               f'({msg.position.x:.2f}, {msg.position.y:.2f})')
        self.queue.append(msg)

    def tick(self):
        now = self.get_clock().now()
        for robot in self.robots.values():
            if robot.state == 'measuring' and now >= robot.measure_until:
                self.finish(robot)
        if self.map is not None:
            self.assign_pending()

    def nearest_available(self, task):
        free = [robot for robot in self.robots.values()
                if robot.state == 'idle' and robot.position is not None
                and task.id not in robot.failed and self.nav[robot.name].server_is_ready()]
        return min(free, key=lambda robot: math.hypot(robot.position.x - task.position.x,
                                                    robot.position.y - task.position.y), default=None)

    def assign_pending(self):
        while self.queue:
            task = self.queue[0]
            if all(task.id in robot.failed for robot in self.robots.values()):
                self.get_logger().warn(f'#{task.id}: both robots failed; no confirmation')
                self.queue.pop(0)
                continue
            robot = self.nearest_available(task)
            if robot is None:
                return
            self.queue.pop(0)
            self.send(robot, task)

    def send(self, r, task):
        r.task = task
        r.goal = self.free_goal(task.position.x, task.position.y)
        if r.goal is None:
            self.fail(r, 'no clear goal near the candidate')
            return
        gx, gy = r.goal
        goal = self.navigation_goal(gx, gy)
        r.state = 'driving'
        self.get_logger().info(f'#{task.id}: sending {r.name} to ({gx:.2f}, {gy:.2f})')
        self.nav[r.name].send_goal_async(goal).add_done_callback(
            lambda f, r=r: self.on_goal_response(r, f))

    def navigation_goal(self, x, y):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.w = 1.0
        return goal

    def on_goal_response(self, r, future):
        try:
            handle = future.result()
        except Exception as error:
            self.fail(r, f'could not send goal: {error}')
            return
        if not handle.accepted:
            self.fail(r, 'goal rejected')
            return
        handle.get_result_async().add_done_callback(lambda f, r=r: self.on_result(r, f))

    def on_result(self, r, future):
        try:
            status = future.result().status
        except Exception as error:
            self.fail(r, f'could not read navigation result: {error}')
            return
        if status != GoalStatus.STATUS_SUCCEEDED or not self.at_goal(r):
            self.fail(r, f'did not reach the goal (navigation status {status})')
            return
        self.start_measuring(r)

    def at_goal(self, r):
        return (r.position is not None and r.goal is not None
                and math.hypot(r.position.x - r.goal[0], r.position.y - r.goal[1])
                <= self.arrival_tolerance)

    def fail(self, r, reason):
        self.get_logger().warn(f'#{r.task.id}: {r.name}: {reason}; trying another robot')
        r.failed.add(r.task.id)
        self.queue.append(r.task)
        self.release_robot(r)

    def start_measuring(self, r):
        r.state = 'measuring'
        r.readings = []
        r.measure_started = self.get_clock().now()
        r.measure_until = r.measure_started + Duration(seconds=self.measure_time)

    def confirmation_message(self, robot):
        task = robot.task
        msg = Detection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.id = task.id
        msg.hazard_type = task.hazard_type
        msg.position = robot.position
        msg.value = sum(robot.readings) / len(robot.readings)
        msg.robot = robot.name
        msg.confirmed = msg.value >= self.confirm[task.hazard_type]
        return msg

    @staticmethod
    def release_robot(robot):
        robot.state, robot.task, robot.goal = 'idle', None, None
        robot.readings = []
        robot.measure_started = robot.measure_until = None

    def finish(self, robot):
        if len(robot.readings) < self.min_samples or not self.at_goal(robot):
            self.fail(robot, 'not enough measurements at the goal')
            return
        msg = self.confirmation_message(robot)
        self.pub.publish(msg)
        unit = 'ppm' if msg.hazard_type == 'gas' else 'uSv/h'
        self.get_logger().info(f'#{msg.id} {"CONFIRMED" if msg.confirmed else "not confirmed"} '
                               f'by {robot.name}: {msg.value:.2f} {unit} on the ground')
        self.release_robot(robot)

    def free_goal(self, x, y):
        """A nearby map cell with clearance, or None if no suitable cell exists."""
        m = self.map
        if m is None:
            return None
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        w, h = m.info.width, m.info.height
        cells = np.array(m.data, dtype=np.int16).reshape(h, w)
        blocked = cells != 0  # occupied or unknown
        k = int(math.ceil(self.clearance / res))
        ci, cj = int((x - ox) / res), int((y - oy) / res)
        cell = find_clear_cell(blocked, ci, cj, k, int(3.0 / res))
        if cell is None:
            return None
        i, j = cell
        return ox + (i + 0.5) * res, oy + (j + 0.5) * res


def cell_is_clear(blocked, i, j, padding):
    height, width = blocked.shape
    if i - padding < 0 or j - padding < 0 or i + padding >= width or j + padding >= height:
        return False
    return not blocked[j - padding:j + padding + 1, i - padding:i + padding + 1].any()


def find_clear_cell(blocked, ci, cj, padding, max_radius):
    for radius in range(max_radius):
        best = None
        for j in range(cj - radius, cj + radius + 1):
            for i in range(ci - radius, ci + radius + 1):
                if max(abs(i - ci), abs(j - cj)) != radius:
                    continue
                if not cell_is_clear(blocked, i, j, padding):
                    continue
                distance = (i - ci) ** 2 + (j - cj) ** 2
                if best is None or distance < best[0]:
                    best = (distance, i, j)
        if best is not None:
            return best[1], best[2]
    return None


def main():
    run_node(Coordinator)
