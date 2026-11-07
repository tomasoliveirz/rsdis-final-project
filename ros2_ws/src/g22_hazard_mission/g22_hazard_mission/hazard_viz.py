"""Hazard map for RViz.

The area is split into cells coloured by the worst reading seen in them (green,
yellow or red). It also shows each robot with its last reading, the sources
estimated by the UAV (orange), their ground checks (red if confirmed, grey if
not), the signs seen by the camera and the real sources when show_truth is true.
"""
import math

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from g22_hazard_mission import run_node

from g22_hazard_interfaces.msg import Detection, SensorReading, Sign
from g22_hazard_mission import field

SAFE = ColorRGBA(r=0.1, g=0.8, b=0.2, a=0.35)
CAUTION = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.5)
DANGER = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.6)
GAS = ColorRGBA(r=0.5, g=0.9, b=0.1, a=0.8)
RADIATION = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.8)
ESTIMATE = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)
CONFIRMED = ColorRGBA(r=0.9, g=0.0, b=0.0, a=0.9)
REJECTED = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.9)
WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


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
        self.signs = {}
        self.estimates = {}      # id -> Detection from the UAV
        self.confirmations = {}  # id -> Detection from a UGV

        for r in robots:
            self.create_subscription(SensorReading, f'/{r}/reading', self.on_reading, 10)
        self.create_subscription(Detection, '/hazards/detections',
                                 self.on_estimate, 10)
        self.create_subscription(Detection, '/hazards/confirmations',
                                 self.on_confirmation, 10)
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Sign, '/signs', self.on_sign, latched)
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

    def on_sign(self, msg):
        self.signs[msg.id] = msg

    def on_estimate(self, msg):
        self.estimates[msg.id] = msg

    def on_confirmation(self, msg):
        self.confirmations[msg.id] = msg

    def marker(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def text_marker(self, namespace, marker_id, position, text, color=WHITE, size=0.4):
        marker = self.marker(namespace, marker_id, Marker.TEXT_VIEW_FACING)
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
        marker.scale.z = size
        marker.color = color
        marker.text = text
        return marker

    def zone_markers(self):
        zones = self.marker('zones', 0, Marker.CUBE_LIST)
        zones.scale.x = zones.scale.y = self.cell
        zones.scale.z = 0.02
        for (i, j), level in self.cells.items():
            zones.points.append(Point(x=(i + 0.5) * self.cell, y=(j + 0.5) * self.cell, z=0.01))
            zones.colors.append((SAFE, CAUTION, DANGER)[level])
        return [zones] if zones.points else []

    def robot_markers(self):
        markers = []
        for index, (name, reading) in enumerate(sorted(self.robots.items())):
            position = reading.position
            dot = self.marker('robots', index, Marker.SPHERE)
            dot.pose.position = position
            dot.scale.x = dot.scale.y = dot.scale.z = 0.4
            dot.color = ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.9)
            text = f'{name}  {reading.gas_ppm:.1f} ppm  {reading.radiation_usvh:.2f} uSv/h'
            label = self.text_marker('robot_labels', index,
                                     (position.x, position.y - 1.7, position.z + 0.8), text, size=0.35)
            markers.extend([dot, label])
        return markers

    def sign_markers(self):
        markers = []
        for index, sign in enumerate(sorted(self.signs.values(), key=lambda s: s.id)):
            gas = sign.type == Sign.GAS
            x, y = sign.position.x, sign.position.y
            square = self.marker('signs', index, Marker.CUBE)
            square.pose.position.x, square.pose.position.y, square.pose.position.z = x, y, 1.2
            square.scale.x = square.scale.y = 0.8
            square.scale.z = 0.05
            square.color = GAS if gas else RADIATION
            label = self.text_marker('sign_labels', index, (x, y, 2.0),
                                     'gas sign' if gas else 'radiation sign', size=0.5)
            markers.extend([square, label])
        return markers

    def truth_markers(self):
        markers = []
        for index, source in enumerate(self.truth):
            marker = self.marker('truth', index, Marker.CYLINDER)
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = source.x, source.y, 0.5
            marker.scale.x = marker.scale.y = 0.5
            marker.scale.z = 1.0
            marker.color = ColorRGBA(r=0.6, g=0.0, b=0.8, a=0.8)
            markers.append(marker)
        return markers

    def estimate_markers(self):
        markers = []
        for hazard_id, estimate in sorted(self.estimates.items()):
            x, y = estimate.position.x, estimate.position.y
            unit = 'ppm' if estimate.hazard_type == 'gas' else 'uSv/h'
            dot = self.marker('estimates', hazard_id, Marker.SPHERE)
            dot.pose.position.x, dot.pose.position.y, dot.pose.position.z = x, y, 0.3
            dot.scale.x = dot.scale.y = dot.scale.z = 0.6
            dot.color = ESTIMATE
            label = self.text_marker('estimate_labels', hazard_id, (x, y + 0.9, 1.2),
                                     f'#{hazard_id} {estimate.hazard_type}: UAV {estimate.value:.1f} {unit}',
                                     color=ESTIMATE)
            markers.extend([dot, label])
        return markers

    def confirmation_markers(self):
        markers = []
        for hazard_id, estimate in sorted(self.estimates.items()):
            confirmation = self.confirmations.get(hazard_id)
            if confirmation is None:
                continue
            x, y = confirmation.position.x, confirmation.position.y
            unit = 'ppm' if estimate.hazard_type == 'gas' else 'uSv/h'
            ring = self.marker('confirmations', hazard_id, Marker.CYLINDER)
            ring.pose.position.x, ring.pose.position.y, ring.pose.position.z = x, y, 0.05
            ring.scale.x = ring.scale.y = 1.2
            ring.scale.z = 0.1
            ring.color = CONFIRMED if confirmation.confirmed else REJECTED
            status = 'CONFIRMED' if confirmation.confirmed else 'NOT confirmed'
            text = f'{status}: {confirmation.robot} {confirmation.value:.1f} {unit}'
            label = self.text_marker('confirmation_labels', hazard_id, (x, y - 0.9, 1.9),
                                     text, color=ring.color)
            markers.extend([ring, label])
        return markers

    def publish(self):
        markers = [*self.zone_markers(), *self.robot_markers(), *self.sign_markers(),
                   *self.truth_markers(), *self.estimate_markers(), *self.confirmation_markers()]
        self.pub.publish(MarkerArray(markers=markers))


def main():
    run_node(HazardViz)
