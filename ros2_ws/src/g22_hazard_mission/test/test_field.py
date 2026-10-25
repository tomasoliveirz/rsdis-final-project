import os
import random

from g22_hazard_mission import field

HAZARDS = os.path.join(os.path.dirname(__file__), '..', 'config', 'hazards.yaml')


def test_load():
    f = field.load(HAZARDS)
    assert {s.type for s in f.sources} == {'gas', 'radiation'}


def test_background_far_away():
    f = field.load(HAZARDS)
    assert abs(f.gas(-14, -11, 0) - f.gas_background) < 0.1
    assert abs(f.radiation(-14, -11, 0) - f.radiation_background) < 0.2


def test_peaks_at_sources():
    f = field.load(HAZARDS)
    for s in f.sources:
        value = f.gas if s.type == 'gas' else f.radiation
        here = value(s.x, s.y, 0)
        assert here > value(s.x + 2, s.y, 0) > value(s.x + 4, s.y, 0)


def test_weaker_with_height():
    f = field.load(HAZARDS)
    s = next(s for s in f.sources if s.type == 'gas')
    assert f.gas(s.x, s.y, 6) < f.gas(s.x, s.y, 0)


def test_noise_is_small():
    f = field.load(HAZARDS)
    rng = random.Random(1)
    s = f.sources[0]
    readings = [f.measure(s.x, s.y, 0, rng)[0] for _ in range(200)]
    mean = sum(readings) / len(readings)
    assert abs(mean - f.gas(s.x, s.y, 0)) / f.gas(s.x, s.y, 0) < 0.02
