"""Sign detection on crops of real images from the UAV camera at 6 m."""
import math
import os

import cv2
import pytest

from g22_hazard_mission.signs import GAS, RADIATION, find_signs, pixel_to_ground

DATA = os.path.join(os.path.dirname(__file__), 'data')


def kinds(name):
    return [s[0] for s in find_signs(cv2.imread(os.path.join(DATA, name + '.png')))]


def test_three_gas_lids():
    assert kinds('gas_lids') == [GAS] * 3


def test_gas_lids_in_the_shade_of_the_tank():
    found = kinds('gas_lids_in_shade')
    assert len(found) >= 2 and set(found) == {GAS}


def test_radiation_lids():
    found = kinds('radiation_lids')
    assert found and set(found) == {RADIATION}


def test_radiation_seen_from_the_side():
    assert RADIATION in kinds('radiation_side_view')


# things that look a bit like signs but are not: yellow pad, red container, a
# black ring on dull yellow, an all-black diamond and an empty red diamond
@pytest.mark.parametrize('name', ['landing_pad', 'red_container', 'lookalike_ring',
                                  'lookalike_black_diamond', 'lookalike_empty_diamond'])
def test_no_sign(name):
    assert kinds(name) == []


LEVEL = (1.0, 0.0, 0.0, 0.0)
W, H, FOV = 1280, 960, 1.396


def test_image_centre_is_below_a_level_camera():
    assert pixel_to_ground(W / 2, H / 2, W, H, FOV, (3.0, 4.0, 5.0), LEVEL, 0.0) == \
        pytest.approx((3.0, 4.0))


def test_image_top_is_forward_and_left_is_plus_y():
    x, y = pixel_to_ground(W / 2, 0, W, H, FOV, (0.0, 0.0, 5.0), LEVEL, 0.0)
    assert x > 0 and y == pytest.approx(0.0)
    x, y = pixel_to_ground(0, H / 2, W, H, FOV, (0.0, 0.0, 5.0), LEVEL, 0.0)
    assert y == pytest.approx(5.0 * math.tan(FOV / 2)) and x == pytest.approx(0.0)


def test_yaw_turns_the_image():
    yaw90 = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
    x, y = pixel_to_ground(W / 2, 0, W, H, FOV, (0.0, 0.0, 5.0), yaw90, 0.0)
    assert y > 0 and x == pytest.approx(0.0, abs=1e-9)


def test_pitch_is_taken_into_account():
    # nose down by 10 degrees: the image centre looks behind the UAV
    a = math.radians(10)
    pitch = (math.cos(a / 2), 0.0, math.sin(a / 2), 0.0)
    x, y = pixel_to_ground(W / 2, H / 2, W, H, FOV, (0.0, 0.0, 5.0), pitch, 0.0)
    assert x == pytest.approx(-5.0 * math.tan(a)) and y == pytest.approx(0.0, abs=1e-9)


def test_camera_below_the_sign_plane_does_not_project_behind_itself():
    assert pixel_to_ground(W / 2, H / 2, W, H, FOV, (0.0, 0.0, 0.5), LEVEL, 0.9) is None
