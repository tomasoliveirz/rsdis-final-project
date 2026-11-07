"""Simulated gas/radiation sensor, one per robot (runs in the robot namespace).

The simulated field is sampled at the actual Gazebo position. The position sent
with the reading is the robot's navigation estimate (odometry for the UAV,
AMCL for the UGVs). Localisation error must not change what a sensor measures.
"""
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformException

from g22_hazard_mission import run_node

from g22_hazard_interfaces.msg import SensorReading
from g22_hazard_mission import field


class HazardSensor(Node):

    def __init__(self):
        super().__init__('hazard_sensor')
        self.robot = self.declare_parameter('robot', 'ugv1').value
        self.pose_source = self.declare_parameter('pose_source', 'tf').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        rate = self.declare_parameter('rate', 2.0).value
        hazards_file = self.declare_parameter('hazards_file', '').value

        self.field = field.load(hazards_file)
        self.position = None

        topic = 'odom' if self.pose_source == 'odom' else 'ground_truth'
        self.create_subscription(Odometry, topic, self.on_odom, 10)
        if self.pose_source != 'odom':
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(SensorReading, 'reading', 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'{self.robot}: {len(self.field.sources)} sources, pose from {self.pose_source}')

    def on_odom(self, msg):
        self.position = msg.pose.pose.position

    def lookup_position(self):
        if self.pose_source == 'odom':
            return self.position
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return None
        return t.transform.translation

    def tick(self):
        p = self.lookup_position()
        if p is None or self.position is None:
            return
        actual = self.position
        gas, rad = self.field.measure(actual.x, actual.y, actual.z)
        msg = SensorReading()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.robot = self.robot
        msg.position.x, msg.position.y, msg.position.z = p.x, p.y, p.z
        msg.gas_ppm = float(gas)
        msg.radiation_usvh = float(rad)
        self.pub.publish(msg)


def main():
    run_node(HazardSensor)
