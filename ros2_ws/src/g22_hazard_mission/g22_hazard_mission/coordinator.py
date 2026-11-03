"""Coordinator: sends the closest free ground robot to each hazard found by the UAV.

For each estimate on /hazards/detections:
  1. pick the free UGV with the lowest cost: straight-line distance to the
     estimate plus task_penalty metres per task it has already done, so both
     robots get work,
  2. move the goal to the nearest free cell of the Nav2 map if the estimate is
     inside or too close to an obstacle,
  3. send the goal to that robot's Nav2 (navigate_to_pose action),
  4. once it is there, average its ground readings for a few seconds and
     publish the result on /hazards/confirmations.
Estimates wait in a queue while both robots are busy.
"""
import math

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid

from g22_hazard_interfaces.msg import Detection, SensorReading


class Robot:
    def __init__(self, name):
        self.name = name
        self.position = None   # from its last SensorReading (map frame)
        self.state = 'idle'    # idle, driving, measuring
        self.task = None       # Detection being checked
        self.done = 0          # tasks finished
        self.readings = []     # values collected while measuring
        self.measure_until = None


class Coordinator(Node):

    def __init__(self):
        super().__init__('coordinator')
        names = self.declare_parameter('robots', ['ugv1', 'ugv2']).value
        map_topic = self.declare_parameter('map_topic', '/ugv1/map').value
        self.clearance = self.declare_parameter('goal_clearance', 0.5).value
        self.measure_time = self.declare_parameter('measure_time', 3.0).value
        self.task_penalty = self.declare_parameter('task_penalty', 5.0).value
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

    # ------------------------------------------------------------ inputs ---

    def on_map(self, msg):
        self.map = msg

    def on_reading(self, name, msg):
        r = self.robots[name]
        r.position = msg.position
        if r.state == 'measuring':
            r.readings.append(msg.gas_ppm if r.task.hazard_type == 'gas' else msg.radiation_usvh)

    def on_detection(self, msg):
        self.get_logger().info(f'#{msg.id} {msg.hazard_type} reported by {msg.robot} at '
                               f'({msg.position.x:.2f}, {msg.position.y:.2f})')
        self.queue.append(msg)

    # -------------------------------------------------------------- tasks ---

    def tick(self):
        now = self.get_clock().now()
        for r in self.robots.values():
            if r.state == 'measuring' and now >= r.measure_until:
                self.finish(r)
        while self.queue:
            task = self.queue[0]
            free = [r for r in self.robots.values() if r.state == 'idle' and r.position is not None]
            if not free:
                return
            # distance plus a penalty per task already done, so the work is shared
            best = min(free, key=lambda r: math.hypot(r.position.x - task.position.x,
                                                      r.position.y - task.position.y)
                       + self.task_penalty * r.done)
            self.queue.pop(0)
            self.send(best, task)

    def send(self, r, task):
        gx, gy = self.free_goal(task.position.x, task.position.y)
        client = self.nav[r.name]
        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f'{r.name}: Nav2 not ready, task #{task.id} back in the queue')
            self.queue.insert(0, task)
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
        goal.pose.pose.orientation.w = 1.0
        r.state, r.task = 'driving', task
        self.get_logger().info(f'#{task.id}: sending {r.name} to ({gx:.2f}, {gy:.2f})')
        client.send_goal_async(goal).add_done_callback(lambda f, r=r: self.on_goal_response(r, f))

    def on_goal_response(self, r, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'{r.name}: goal rejected, measuring where it is')
            self.start_measuring(r)
            return
        handle.get_result_async().add_done_callback(lambda f, r=r: self.on_result(r, f))

    def on_result(self, r, future):
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'{r.name}: navigation ended with status {status}, '
                                   'measuring where it is')
        self.start_measuring(r)

    def start_measuring(self, r):
        r.state = 'measuring'
        r.readings = []
        r.measure_until = self.get_clock().now() + Duration(seconds=self.measure_time)

    def finish(self, r):
        task = r.task
        value = float(np.mean(r.readings)) if r.readings else 0.0
        msg = Detection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.id = task.id
        msg.hazard_type = task.hazard_type
        if r.position is not None:
            msg.position = r.position
        msg.value = value
        msg.robot = r.name
        msg.confirmed = value >= self.confirm[task.hazard_type]
        self.pub.publish(msg)
        unit = 'ppm' if task.hazard_type == 'gas' else 'uSv/h'
        self.get_logger().info(f'#{task.id} {"CONFIRMED" if msg.confirmed else "not confirmed"} '
                               f'by {r.name}: {value:.2f} {unit} on the ground')
        r.state, r.task = 'idle', None
        r.done += 1

    # ---------------------------------------------------------------- map ---

    def free_goal(self, x, y):
        """Nearest map cell to (x, y) with no obstacle within `clearance` (or (x, y))."""
        m = self.map
        if m is None:
            return x, y
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        w, h = m.info.width, m.info.height
        cells = np.array(m.data, dtype=np.int16).reshape(h, w)
        blocked = cells != 0  # occupied or unknown
        k = int(math.ceil(self.clearance / res))
        ci, cj = int((x - ox) / res), int((y - oy) / res)
        for radius in range(0, int(3.0 / res)):  # search rings up to 3 m away
            best = None
            for j in range(cj - radius, cj + radius + 1):
                for i in range(ci - radius, ci + radius + 1):
                    if max(abs(i - ci), abs(j - cj)) != radius:
                        continue
                    if i - k < 0 or j - k < 0 or i + k >= w or j + k >= h:
                        continue
                    if blocked[j - k:j + k + 1, i - k:i + k + 1].any():
                        continue
                    d = (i - ci) ** 2 + (j - cj) ** 2
                    if best is None or d < best[0]:
                        best = (d, i, j)
            if best:
                _, i, j = best
                return ox + (i + 0.5) * res, oy + (j + 0.5) * res
        return x, y


def main():
    rclpy.init()
    node = Coordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
