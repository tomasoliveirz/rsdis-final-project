from g22_hazard_mission.search import lawnmower


def test_lawnmower_lanes():
    route = lawnmower(-13, 13, -10, 10, 6)
    assert route == [(-13, -10), (13, -10), (13, -4), (-13, -4),
                     (-13, 2), (13, 2), (13, 8), (-13, 8)]


def test_lawnmower_includes_last_lane():
    route = lawnmower(0, 1, 0, 4, 2)
    assert {y for _, y in route} == {0, 2, 4}


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
