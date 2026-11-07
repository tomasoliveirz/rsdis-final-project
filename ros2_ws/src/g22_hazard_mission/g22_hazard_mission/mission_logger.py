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

from rclpy.node import Node

from g22_hazard_mission import run_node

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

        self.create_subscription(Detection, '/hazards/detections',
                                 lambda m: self.log('estimate', m), 10)
        self.create_subscription(Detection, '/hazards/confirmations',
                                 lambda m: self.log('confirmation', m), 10)
        self.start_timer = self.create_timer(0.5, self.check_start)
        self.get_logger().info(f'logging to {self.path}')

    def check_start(self):
        # the simulated clock is 0 until the first /clock message arrives
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.start is None and t > 0:
            self.start = t
        if self.start is not None:
            self.start_timer.cancel()

    def now(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.start is None:
            self.start = t
        return t - self.start

    def reference_source(self, detection):
        sources = [source for source in self.sources if source.type == detection.hazard_type]
        return min(sources, key=lambda source: math.hypot(source.x - detection.position.x,
                                                        source.y - detection.position.y), default=None)

    @staticmethod
    def event_row(event, detection, source, elapsed, error):
        return [f'{elapsed:.1f}', event, detection.id, detection.hazard_type, detection.robot,
                f'{detection.position.x:.2f}', f'{detection.position.y:.2f}', f'{detection.value:.3f}',
                int(detection.confirmed), source.name, f'{error:.2f}']

    def log(self, event, detection):
        source = self.reference_source(detection)
        if source is None:
            self.get_logger().warn(f'no reference source for {detection.hazard_type}')
            return
        error = math.hypot(source.x - detection.position.x, source.y - detection.position.y)
        elapsed = self.now()
        self.csv.writerow(self.event_row(event, detection, source, elapsed, error))
        self.file.flush()
        self.get_logger().info(
            f'{event} #{detection.id}: nearest {source.name}, error {error:.2f} m, t = {elapsed:.0f} s')

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main():
    run_node(MissionLogger)
