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
def test_grid_has_nine_points_around_centre():
    from g22_hazard_mission.search import grid
    pts = grid(1, 2, 3)
    assert len(pts) == 9 and (1, 2) in pts and (4, 5) in pts


def test_estimate_finds_both_source_types():
    import os
    import random
    from g22_hazard_mission import field
    from g22_hazard_mission.search import estimate_source, grid
    f = field.load(os.path.join(os.path.dirname(__file__), '..', 'config', 'hazards.yaml'))
    rng = random.Random(3)
    for s in f.sources:
        samples = []
        for x, y in grid(s.x + 1.5, s.y - 1.0, 3.0):
            for _ in range(5):
                g, r = f.measure(x, y, 6.0, rng)
                samples.append((x, y, g if s.type == 'gas' else r))
        bg = f.gas_background if s.type == 'gas' else f.radiation_background
        ex, ey = estimate_source(samples, s.type, bg)
        assert ((ex - s.x) ** 2 + (ey - s.y) ** 2) ** 0.5 < 0.8
