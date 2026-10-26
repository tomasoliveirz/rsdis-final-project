from g22_hazard_mission.search import lawnmower


def test_lawnmower_lanes():
    route = lawnmower(-13, 13, -10, 10, 6)
    assert route == [(-13, -10), (13, -10), (13, -4), (-13, -4),
                     (-13, 2), (13, 2), (13, 8), (-13, 8)]


def test_lawnmower_includes_last_lane():
    route = lawnmower(0, 1, 0, 4, 2)
    assert {y for _, y in route} == {0, 2, 4}
