import math
import os
import random

from g22_hazard_mission import field
from g22_hazard_mission.search import CompassSearch, lawnmower

HAZARDS = os.path.join(os.path.dirname(__file__), '..', 'config', 'hazards.yaml')


def test_lawnmower_lanes():
    route = lawnmower(-13, 13, -10, 10, 6)
    assert route == [(-13, -10), (13, -10), (13, -4), (-13, -4),
                     (-13, 2), (13, 2), (13, 8), (-13, 8)]


def test_lawnmower_includes_last_lane():
    route = lawnmower(0, 1, 0, 4, 2)
    assert {y for _, y in route} == {0, 2, 4}


def test_compass_search_finds_a_peak():
    search = CompassSearch(3.0, -2.0, step=2.0, min_step=0.5)
    while not search.done:
        x, y = search.target
        search.add(-(x - 1.0) ** 2 - (y - 1.0) ** 2)
    assert math.hypot(search.x - 1.0, search.y - 1.0) < 0.5


def test_compass_search_stops_when_probes_run_out():
    search = CompassSearch(0.0, 0.0, step=2.0, min_step=0.01, max_probes=9)
    probes = 0
    while not search.done:
        x, y = search.target
        search.add(-(x - 50) ** 2)
        probes += 1
    assert probes <= 9


def test_uav_finds_every_source_from_the_edge_of_its_plume():
    """Same settings as uav_survey: 6 m altitude, 5 readings averaged per probe."""
    f = field.load(HAZARDS)
    rng = random.Random(0)
    for s in f.sources:
        search = CompassSearch(s.x - 4.0, s.y + 1.0, step=2.0, min_step=0.5)
        while not search.done:
            x, y = search.target
            readings = [f.measure(x, y, 6.0, rng)[0 if s.type == 'gas' else 1] for _ in range(5)]
            search.add(sum(readings) / len(readings))
        assert math.hypot(search.x - s.x, search.y - s.y) < 1.5, s.name


def test_search_stays_inside_the_survey_area():
    search = CompassSearch(0.0, 0.0, step=2.0, min_step=0.5,
                           bounds=(-1.0, 1.0, -1.0, 1.0))
    probes = 0
    while not search.done:
        x, y = search.target
        assert -1.0 <= x <= 1.0 and -1.0 <= y <= 1.0
        search.add(x + y)
        probes += 1
        assert probes <= 40


def test_search_needs_neither_background_nor_a_gaussian_field():
    search = CompassSearch(-3.0, -2.0, step=2.0, min_step=0.5)
    while not search.done:
        x, y = search.target
        search.add(137.0 + max(0.0, 40.0 - abs(x - 1.0) - 2 * abs(y - 2.0)))
    assert math.hypot(search.x - 1.0, search.y - 2.0) <= 0.5


def test_survey_finishes_after_searching_the_edge_of_an_existing_plume():
    from types import SimpleNamespace
    import rclpy
    from geometry_msgs.msg import Point, Pose
    from g22_hazard_interfaces.msg import SensorReading
    from g22_hazard_mission.uav_survey import UavSurvey

    rclpy.init(args=[])
    node = UavSurvey()
    try:
        node.pose = Pose(position=Point(x=-12.0, y=-10.0, z=0.15))
        node.home = (-12.0, -10.0)
        reports = []
        node.detection_pub = SimpleNamespace(publish=reports.append)

        def move(x, y, z):
            p = node.pose.position
            dx, dy = x - p.x, y - p.y
            distance = math.hypot(dx, dy)
            step = min(distance, node.max_speed * 0.1)
            if distance:
                p.x += step * dx / distance
                p.y += step * dy / distance
            p.z += max(-0.1, min(0.1, z - p.z))
            return math.hypot(x - p.x, y - p.y) < node.tolerance and abs(z - p.z) < node.tolerance

        node.fly_to = move
        readings = field.load(HAZARDS)
        rng = random.Random(20)
        for _ in range(5000):
            node.tick()
            p = node.pose.position
            gas, rad = readings.measure(p.x, p.y, p.z, rng)
            node.on_reading(SensorReading(position=p, gas_ppm=gas, radiation_usvh=rad))
            if node.state == 'done':
                break
        assert node.state == 'done'
        assert node.mission_timer.is_canceled()
        assert len(reports) == 3
        for candidate in reports:
            assert -13 <= candidate.position.x <= 13
            assert -10 <= candidate.position.y <= 10
    finally:
        node.destroy_node()
        rclpy.shutdown()
