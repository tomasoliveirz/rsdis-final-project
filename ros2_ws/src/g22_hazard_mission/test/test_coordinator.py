from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import Point
from rclpy.time import Time

from g22_hazard_interfaces.msg import Detection
from g22_hazard_mission.coordinator import Coordinator, Robot


@pytest.fixture
def coordinator():
    node = Coordinator.__new__(Coordinator)
    node.robots = {name: Robot(name) for name in ('ugv1', 'ugv2')}
    for robot in node.robots.values():
        robot.position = Point(x=0.0, y=0.0)
    node.queue = []
    node.map = object()
    node.clearance = 0.5
    node.measure_time = 3.0
    node.min_samples = 3
    node.arrival_tolerance = 0.75
    node.confirm = {'gas': 5.0, 'radiation': 0.5}
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=10))
    node.get_logger = lambda: Mock()
    node.free_goal = lambda x, y: (x, y)
    node.pub = Mock()
    node.nav = {name: Mock() for name in node.robots}
    for client in node.nav.values():
        client.server_is_ready.return_value = True
    return node


def task():
    return Detection(id=1, hazard_type='gas', position=Point(x=4.0, y=0.0))


def test_nearest_free_robot_is_chosen_regardless_of_previous_work(coordinator):
    near, far = coordinator.robots.values()
    near.position.x = 3.0
    far.position.x = 0.0
    near.task = task()
    near.goal = (3.0, 0.0)
    near.readings = [6.0, 6.0, 6.0]
    coordinator.finish(near)
    coordinator.queue = [task()]
    coordinator.tick()
    assert near.state == 'driving'
    assert far.state == 'idle'


def test_unavailable_nav2_returns_control_without_retry_loop(coordinator):
    for client in coordinator.nav.values():
        client.server_is_ready.return_value = False
        client.wait_for_server.side_effect = AssertionError('blocking Nav2 wait inside tick')
    coordinator.queue = [task()]
    coordinator.tick()
    assert len(coordinator.queue) == 1
    assert all(r.state == 'idle' for r in coordinator.robots.values())


def test_busy_nearest_robot_does_not_receive_another_goal(coordinator):
    near, far = coordinator.robots.values()
    near.position.x = 3.0
    near.state = 'driving'
    coordinator.queue = [task()]
    coordinator.tick()
    assert far.state == 'driving'


def test_navigation_failure_reassigns_without_confirming(coordinator):
    near, far = coordinator.robots.values()
    near.task = task()
    near.state = 'driving'
    near.goal = (4.0, 0.0)
    coordinator.on_result(near, SimpleNamespace(result=lambda: SimpleNamespace(status=6)))
    coordinator.pub.publish.assert_not_called()
    assert near.state == 'idle'
    assert near.task is None
    coordinator.tick()
    assert far.state == 'driving'
    assert far.task.id == 1


def test_rejected_goal_does_not_start_measurement(coordinator):
    robot = coordinator.robots['ugv1']
    robot.task, robot.state = task(), 'driving'
    coordinator.on_goal_response(robot, SimpleNamespace(result=lambda: SimpleNamespace(accepted=False)))
    assert robot.state == 'idle'
    coordinator.pub.publish.assert_not_called()
    assert len(coordinator.queue) == 1


def test_succeeded_goal_far_from_latest_position_is_not_confirmation(coordinator):
    robot = coordinator.robots['ugv1']
    robot.task, robot.state, robot.goal = task(), 'driving', (4.0, 0.0)
    coordinator.on_result(robot, SimpleNamespace(result=lambda: SimpleNamespace(status=4)))
    assert robot.state == 'idle'
    coordinator.pub.publish.assert_not_called()


def test_missing_readings_are_not_reported_as_zero(coordinator):
    robot = coordinator.robots['ugv1']
    robot.task, robot.state, robot.goal = task(), 'measuring', (4.0, 0.0)
    robot.position.x = 4.0
    coordinator.finish(robot)
    coordinator.pub.publish.assert_not_called()
    assert len(coordinator.queue) == 1


def test_successful_ground_check_uses_average_and_original_id(coordinator):
    robot = coordinator.robots['ugv1']
    robot.task, robot.state, robot.goal = task(), 'measuring', (4.0, 0.0)
    robot.position.x = 4.0
    robot.readings = [6.0, 8.0, 7.0]
    coordinator.finish(robot)
    result = coordinator.pub.publish.call_args.args[0]
    assert result.id == 1 and result.confirmed
    assert result.value == pytest.approx(7.0)
    assert robot.state == 'idle'


def test_no_task_is_assigned_before_the_map_arrives(coordinator):
    coordinator.map = None
    coordinator.queue = [task()]
    coordinator.tick()
    assert len(coordinator.queue) == 1
    assert all(r.state == 'idle' for r in coordinator.robots.values())


def test_failed_measurement_is_not_retried_forever_on_both_robots(coordinator):
    for robot in coordinator.robots.values():
        robot.failed.add(1)
    coordinator.queue = [task()]
    coordinator.tick()
    assert not coordinator.queue
    coordinator.pub.publish.assert_not_called()


def test_old_or_out_of_position_readings_are_not_used(coordinator):
    from g22_hazard_interfaces.msg import SensorReading
    robot = coordinator.robots['ugv1']
    robot.task, robot.goal = task(), (4.0, 0.0)
    coordinator.start_measuring(robot)
    reading = SensorReading(position=Point(x=4.0), gas_ppm=10.0)
    reading.header.stamp = Time(seconds=9).to_msg()
    coordinator.on_reading('ugv1', reading)
    assert not robot.readings
    reading.header.stamp = Time(seconds=11).to_msg()
    reading.position.x = 0.0
    coordinator.on_reading('ugv1', reading)
    assert not robot.readings
    reading.position.x = 4.0
    coordinator.on_reading('ugv1', reading)
    assert robot.readings == [10.0]


def test_map_without_clear_space_does_not_return_an_occupied_goal(coordinator):
    from nav_msgs.msg import OccupancyGrid
    grid = OccupancyGrid()
    grid.info.width = grid.info.height = 10
    grid.info.resolution = 1.0
    grid.data = [100] * 100
    coordinator.map = grid
    assert Coordinator.free_goal(coordinator, 4.0, 4.0) is None
