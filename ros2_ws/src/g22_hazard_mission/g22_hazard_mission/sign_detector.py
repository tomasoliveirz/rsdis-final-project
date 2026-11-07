"""Looks for radiation and gas signs in the UAV camera images (OpenCV).

Each sign seen is put on the map from the UAV pose at the time of the image
(position and full orientation, so a tilted UAV is fine). Sightings closer than
merge_radius are the same sign, and a sign is published on /signs once it has
been seen min_sightings times. The image with the detections drawn on it goes
out on camera/signs.
"""
from collections import deque
from dataclasses import dataclass
import math

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import cv2

from g22_hazard_mission import run_node

from g22_hazard_interfaces.msg import Sign
from g22_hazard_mission.signs import GAS, RADIATION, find_signs, pixel_to_ground, rotate

COLOURS = {GAS: (0, 0, 255), RADIATION: (0, 220, 255)}  # BGR
NAMES = {GAS: 'gas', RADIATION: 'radiation'}


@dataclass
class SeenSign:
    type: int
    x: float
    y: float
    sightings: int = 1
    published: bool = False


class SignDetector(Node):

    def __init__(self):
        super().__init__('sign_detector')
        self.hfov = self.declare_parameter('horizontal_fov', 1.396).value
        self.camera_offset = self.declare_parameter('camera_offset', 0.06).value  # below the UAV
        self.sign_height = self.declare_parameter('sign_height', 0.9).value  # barrel lids
        self.merge_radius = self.declare_parameter('merge_radius', 1.5).value
        self.min_sightings = self.declare_parameter('min_sightings', 3).value

        self.bridge = CvBridge()
        self.poses = deque(maxlen=50)  # (stamp, pose), about 1 s of odometry
        self.signs = []

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Image, 'camera/image', self.on_image, 10)
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sign_pub = self.create_publisher(Sign, '/signs', latched)
        self.image_pub = self.create_publisher(Image, 'camera/signs', 10)

    def on_odom(self, msg):
        self.poses.append((Time.from_msg(msg.header.stamp), msg.pose.pose))

    def pose_at(self, stamp):
        if not self.poses:
            return None
        return min(self.poses, key=lambda item: abs((item[0] - stamp).nanoseconds))[1]

    def camera_pose(self, pose):
        position, orientation = pose.position, pose.orientation
        quaternion = (orientation.w, orientation.x, orientation.y, orientation.z)
        offset = rotate(quaternion, (0.0, 0.0, -self.camera_offset))
        camera = (position.x + offset[0], position.y + offset[1], position.z + offset[2])
        return camera, quaternion

    @staticmethod
    def annotate_sign(image, kind, u, v, size):
        half = int(size / 2) + 4
        corner = (int(u) - half, int(v) - half)
        cv2.rectangle(image, corner, (int(u) + half, int(v) + half), COLOURS[kind], 3)
        cv2.putText(image, NAMES[kind], (corner[0], corner[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, COLOURS[kind], 2)

    def on_image(self, msg):
        pose = self.pose_at(Time.from_msg(msg.header.stamp))
        if pose is None:
            return
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        camera, quaternion = self.camera_pose(pose)
        for kind, u, v, size in find_signs(image):
            hit = pixel_to_ground(u, v, image.shape[1], image.shape[0], self.hfov,
                                  camera, quaternion, self.sign_height)
            if hit is not None:
                self.add(kind, *hit)
                self.annotate_sign(image, kind, u, v, size)
        out = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        out.header = msg.header
        self.image_pub.publish(out)

    def add(self, kind, x, y):
        """Average the sighting into a known sign, or start a new one."""
        for s in self.signs:
            if s.type == kind and math.hypot(s.x - x, s.y - y) < self.merge_radius:
                n = s.sightings
                s.x, s.y, s.sightings = (s.x * n + x) / (n + 1), (s.y * n + y) / (n + 1), n + 1
                break
        else:
            s = SeenSign(kind, x, y)
            self.signs.append(s)
        if s.sightings >= self.min_sightings and not s.published:
            s.published = True
            self.publish(s)

    def publish(self, s):
        msg = Sign()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.id = sum(1 for t in self.signs if t.published)
        msg.type = Sign.GAS if s.type == GAS else Sign.RADIATION
        msg.position.x, msg.position.y = s.x, s.y
        msg.seen_by = self.get_namespace().strip('/')
        self.sign_pub.publish(msg)
        self.get_logger().info(f'sign {msg.id}: {NAMES[s.type]} at ({s.x:.1f}, {s.y:.1f})')


def main():
    run_node(SignDetector)
