import csv
import io
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.time import Time

from g22_hazard_interfaces.msg import Detection, SensorReading, Sign
from g22_hazard_mission.field import Source
from g22_hazard_mission.hazard_viz import DANGER, HazardViz
from g22_hazard_mission.mission_logger import MissionLogger
from g22_hazard_mission.sign_detector import SignDetector


@pytest.fixture
def ros_context():
    rclpy.init(args=[])
    yield
    rclpy.shutdown()


def test_rviz_preserves_readings_signs_estimates_and_ground_checks(ros_context):
    node = HazardViz()
    try:
        node.pub = Mock()
        node.on_reading(SensorReading(robot='ugv1', position=Point(x=1.0, y=2.0), gas_ppm=25.0))
        node.on_reading(SensorReading(robot='ugv1', position=Point(x=1.0, y=2.0), gas_ppm=1.0))
        node.on_sign(Sign(id=9, type=Sign.RADIATION, position=Point(x=3.0, y=4.0)))
        node.on_estimate(Detection(id=7, hazard_type='gas', value=12.0,
                                   position=Point(x=5.0, y=6.0)))
        node.on_confirmation(Detection(id=7, hazard_type='gas', robot='ugv2', value=8.0,
                                       confirmed=True, position=Point(x=5.0, y=6.0)))
        node.on_confirmation(Detection(id=99, hazard_type='gas', confirmed=True))
        node.truth = [Source('reference', 'gas', 8.0, 9.0, 50.0)]
        node.publish()
        markers = node.pub.publish.call_args.args[0].markers
        by_key = {(marker.ns, marker.id): marker for marker in markers}
        assert len(markers) == len(by_key) == 10
        assert all(marker.header.frame_id == 'map' for marker in markers)
        assert by_key['zones', 0].colors == [DANGER]
        assert by_key['sign_labels', 0].text == 'radiation sign'
        assert by_key['estimate_labels', 7].text == '#7 gas: UAV 12.0 ppm'
        assert by_key['confirmation_labels', 7].text == 'CONFIRMED: ugv2 8.0 ppm'
        assert ('confirmations', 99) not in by_key
    finally:
        node.destroy_node()


def test_camera_publishes_annotations_and_projected_signs(ros_context):
    node = SignDetector()
    try:
        node.min_sightings = 1
        node.sign_pub, node.image_pub = Mock(), Mock()
        odom = Odometry()
        odom.header.stamp = Time(seconds=10).to_msg()
        odom.pose.pose.position.z = 6.0
        odom.pose.pose.orientation.w = 1.0
        node.on_odom(odom)
        image = cv2.imread(str(Path(__file__).parent / 'data' / 'gas_lids.png'))
        message = CvBridge().cv2_to_imgmsg(image, 'bgr8')
        message.header = odom.header
        node.on_image(message)
        annotated = node.image_pub.publish.call_args.args[0]
        assert annotated.header == message.header
        assert not np.array_equal(CvBridge().imgmsg_to_cv2(annotated, 'bgr8'), image)
        assert node.sign_pub.publish.call_count >= 1
        assert all(call.args[0].type == Sign.GAS for call in node.sign_pub.publish.call_args_list)
    finally:
        node.destroy_node()


def test_camera_waits_for_odometry(ros_context):
    node = SignDetector()
    try:
        node.image_pub = Mock()
        node.on_image(CvBridge().cv2_to_imgmsg(np.zeros((20, 20, 3), np.uint8), 'bgr8'))
        node.image_pub.publish.assert_not_called()
    finally:
        node.destroy_node()


def test_csv_keeps_event_identity_units_and_reference_error():
    node = MissionLogger.__new__(MissionLogger)
    node.sources = [Source('leak', 'gas', 2.0, 0.0, 50.0)]
    node.start = 1.0
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=10))
    node.get_logger = lambda: Mock()
    node.file = io.StringIO()
    node.csv = csv.writer(node.file)
    message = Detection(id=7, hazard_type='gas', robot='uav1', value=12.0,
                        position=Point(x=1.0, y=0.0))
    node.log('estimate', message)
    assert node.file.getvalue() == '9.0,estimate,7,gas,uav1,1.00,0.00,12.000,0,leak,1.00\r\n'


def test_logger_stops_polling_once_the_simulation_clock_has_started():
    node = MissionLogger.__new__(MissionLogger)
    node.start = None
    node.start_timer = Mock()
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=10))
    node.check_start()
    assert node.start == 10.0
    node.start_timer.cancel.assert_called_once()


def test_aerial_reading_does_not_classify_the_ground_as_safe():
    node = HazardViz.__new__(HazardViz)
    node.cells, node.robots, node.cell = {}, {}, 1.0
    reading = SensorReading(robot='uav1', position=Point(x=1.0, y=2.0, z=6.0), gas_ppm=0.5)
    node.on_reading(reading)
    assert not node.cells
    assert node.robots['uav1'] == reading


def test_simulated_sensor_uses_actual_pose_but_reports_navigation_pose():
    from g22_hazard_mission.hazard_sensor import HazardSensor

    node = HazardSensor.__new__(HazardSensor)
    node.robot, node.map_frame = 'ugv1', 'map'
    node.position = Point(x=2.0, y=3.0, z=0.1)
    node.lookup_position = lambda: Point(x=20.0, y=30.0)
    node.field = Mock()
    node.field.measure.return_value = (10.0, 0.2)
    node.pub = Mock()
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=10))
    node.tick()
    node.field.measure.assert_called_once_with(2.0, 3.0, 0.1)
    reading = node.pub.publish.call_args.args[0]
    assert reading.position.x == 20.0 and reading.gas_ppm == 10.0
    node.position = None
    node.pub.reset_mock()
    node.tick()
    node.pub.publish.assert_not_called()
