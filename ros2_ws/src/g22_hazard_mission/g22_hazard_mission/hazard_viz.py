"""Hazard map for RViz.

The area is split into cells coloured by the worst reading seen in them (green,
yellow or red). It also shows each robot with its last reading, and the real
sources when show_truth is true.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from g22_hazard_interfaces.msg import SensorReading
from g22_hazard_mission import field

SAFE = ColorRGBA(r=0.1, g=0.8, b=0.2, a=0.35)
CAUTION = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.5)
DANGER = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.6)


class HazardViz(Node):

    def __init__(self):
        super().__init__('hazard_viz')
        robots = self.declare_parameter('robots', ['uav1', 'ugv1', 'ugv2']).value
        self.frame = self.declare_parameter('map_frame', 'map').value
        self.cell = self.declare_parameter('cell_size', 1.0).value
        self.gas_caution = self.declare_parameter('gas_caution_ppm', 5.0).value
        self.gas_danger = self.declare_parameter('gas_danger_ppm', 20.0).value
        self.rad_caution = self.declare_parameter('radiation_caution_usvh', 0.5).value
        self.rad_danger = self.declare_parameter('radiation_danger_usvh', 2.0).value
        hazards_file = self.declare_parameter('hazards_file', '').value
        show_truth = self.declare_parameter('show_truth', False).value

        self.truth = field.load(hazards_file).sources if show_truth and hazards_file else []
        self.cells = {}   # (i, j) -> worst level (0 safe, 1 caution, 2 danger)
        self.robots = {}  # name -> last SensorReading

        for r in robots:
            self.create_subscription(SensorReading, f'/{r}/reading', self.on_reading, 10)
        self.pub = self.create_publisher(MarkerArray, '/hazards/markers', 10)
        self.create_timer(0.5, self.publish)

    def level(self, msg):
        if msg.gas_ppm >= self.gas_danger or msg.radiation_usvh >= self.rad_danger:
            return 2
        if msg.gas_ppm >= self.gas_caution or msg.radiation_usvh >= self.rad_caution:
            return 1
        return 0

    def on_reading(self, msg):
        key = (math.floor(msg.position.x / self.cell), math.floor(msg.position.y / self.cell))
        self.cells[key] = max(self.cells.get(key, 0), self.level(msg))
        self.robots[msg.robot] = msg

    def marker(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish(self):
        out = MarkerArray()

        zones = self.marker('zones', 0, Marker.CUBE_LIST)
        zones.scale.x = zones.scale.y = self.cell
        zones.scale.z = 0.02
        for (i, j), lvl in self.cells.items():
            zones.points.append(Point(x=(i + 0.5) * self.cell, y=(j + 0.5) * self.cell, z=0.01))
            zones.colors.append((SAFE, CAUTION, DANGER)[lvl])
        if zones.points:
            out.markers.append(zones)

        for k, (name, r) in enumerate(sorted(self.robots.items())):
            dot = self.marker('robots', k, Marker.SPHERE)
            dot.pose.position = r.position
            dot.scale.x = dot.scale.y = dot.scale.z = 0.4
            dot.color = ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.9)
            label = self.marker('robot_labels', k, Marker.TEXT_VIEW_FACING)
            label.pose.position.x = r.position.x
            label.pose.position.y = r.position.y
            label.pose.position.z = r.position.z + 0.8
            label.scale.z = 0.5
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f'{name}  {r.gas_ppm:.1f} ppm  {r.radiation_usvh:.2f} uSv/h'
            out.markers += [dot, label]

        for k, s in enumerate(self.truth):
            t = self.marker('truth', k, Marker.CYLINDER)
            t.pose.position.x, t.pose.position.y, t.pose.position.z = s.x, s.y, 0.5
            t.scale.x = t.scale.y = 0.5
            t.scale.z = 1.0
            t.color = ColorRGBA(r=0.6, g=0.0, b=0.8, a=0.8)
            out.markers.append(t)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = HazardViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
