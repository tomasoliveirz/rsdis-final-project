from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Point
from rclpy.time import Time

from g22_hazard_interfaces.msg import SensorReading
from g22_hazard_mission.hazard_viz import HazardViz


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
