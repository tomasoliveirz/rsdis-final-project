"""Mission metrics: compares each estimate and confirmation with the real sources.

Only this node (and the simulated sensors) read config/hazards.yaml. It writes
one CSV per run in `output_dir` with, for each event, the time since the mission
started (the logger starts with the UAV),
the robot, the position and the distance to the closest real source of the
same type.
"""
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node

from g22_hazard_interfaces.msg import Detection
from g22_hazard_mission import field


class MissionLogger(Node):

    def __init__(self):
        super().__init__('mission_logger')
        hazards_file = self.declare_parameter('hazards_file', '').value
        out_dir = os.path.expanduser(self.declare_parameter('output_dir', '~/g22_runs').value)
        self.sources = field.load(hazards_file).sources
        self.start = None

        os.makedirs(out_dir, exist_ok=True)
        self.path = os.path.join(out_dir, time.strftime('run_%Y%m%d_%H%M%S.csv'))
        self.file = open(self.path, 'w', newline='')
        self.csv = csv.writer(self.file)
        self.csv.writerow(['t_s', 'event', 'id', 'type', 'robot', 'x', 'y', 'value',
                           'confirmed', 'closest_source', 'error_m'])
        self.found = set()
        self.confirmed = set()

        self.create_subscription(Detection, '/hazards/detections',
                                 lambda m: self.log('estimate', m), 10)
        self.create_subscription(Detection, '/hazards/confirmations',
                                 lambda m: self.log('confirmation', m), 10)
        self.create_timer(0.5, self.check_start)
        self.get_logger().info(f'logging to {self.path}')

    def check_start(self):
        # the simulated clock is 0 until the first /clock message arrives
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.start is None and t > 0:
            self.start = t

    def now(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.start is None:
            self.start = t
        return t - self.start

    def log(self, event, m):
        same = [s for s in self.sources if s.type == m.hazard_type] or self.sources
        closest = min(same, key=lambda s: math.hypot(s.x - m.position.x, s.y - m.position.y))
        error = math.hypot(closest.x - m.position.x, closest.y - m.position.y)
        t = self.now()
        self.csv.writerow([f'{t:.1f}', event, m.id, m.hazard_type, m.robot,
                           f'{m.position.x:.2f}', f'{m.position.y:.2f}', f'{m.value:.3f}',
                           int(m.confirmed), closest.name, f'{error:.2f}'])
        self.file.flush()
        if event == 'estimate':
            self.found.add(closest.name)
        elif m.confirmed:
            self.confirmed.add(closest.name)
        self.get_logger().info(
            f'{event} #{m.id} -> {closest.name}, error {error:.2f} m, t = {t:.0f} s '
            f'({len(self.found)}/{len(self.sources)} found, '
            f'{len(self.confirmed)}/{len(self.sources)} confirmed)')

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = MissionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
