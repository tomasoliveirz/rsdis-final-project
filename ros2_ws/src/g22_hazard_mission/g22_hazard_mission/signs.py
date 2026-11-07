"""Finds the radiation and gas signs on the barrel lids in a camera image.

Radiation: a saturated yellow region with part of the black trefoil inside.
Gas: a saturated red outline with a light inside and part of the black flame.
Light and dark are measured against the sign colour, so shade does not matter.
The shape is not checked, so a lid seen at an angle together with the label on
the side still counts. Plain yellow or red things (the landing pad, a red
container) are not saturated enough or have nothing inside, so they are ignored.
OpenCV only, no ROS.
"""
import math

import cv2
import numpy as np

GAS, RADIATION = 0, 1

# OpenCV HSV: H in 0..180, S and V in 0..255. Only hue and saturation pick the
# sign colours, so a sign in the shade is found too. The minimum saturation sits
# between the signs (about 255) and dull yellow or red things (about 130-150).
YELLOW = ((18, 190, 20), (34, 255, 255))
RED = [((0, 190, 20), (8, 255, 255)), ((172, 190, 20), (180, 255, 255))]
# black and white are judged against the brightness of the sign colour itself
DARK = 0.45     # black symbol: darker than this times the sign colour
LIGHT = 0.8     # white inside the diamond: at least this bright
GREY_S = 80     # white has little saturation


def _inside(contour, shape):
    mask = np.zeros(shape[:2], np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)
    return mask > 0


def matches_symbol(kind, contour, mask, hsv):
    saturation, brightness = hsv[..., 1], hsv[..., 2]
    inside = _inside(contour, hsv.shape)
    coloured = inside & (mask > 0)
    reference = np.median(brightness[coloured])
    area = inside.sum()
    dark_part = (inside & (brightness < DARK * reference)).sum() / area
    if kind == RADIATION:
        return 0.15 < dark_part < 0.6
    light = inside & ~coloured & (saturation < GREY_S) & (brightness > LIGHT * reference)
    return light.sum() / area > 0.3 and 0.02 < dark_part < 0.4


def find_signs(bgr, min_area=300):
    """List of (type, u, v, size) in pixels, one per sign seen."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (3, 3), 0), cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, *YELLOW)
    red = cv2.inRange(hsv, *RED[0]) | cv2.inRange(hsv, *RED[1])
    found = []
    for kind, mask in ((RADIATION, yellow), (GAS, red)):
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < min_area or not matches_symbol(kind, contour, mask, hsv):
                continue
            x, y, width, height = cv2.boundingRect(contour)
            found.append((kind, x + width / 2, y + height / 2, max(width, height)))
    return found


def rotate(q, v):
    """Rotate vector v by the unit quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * (q_xyz x v), v' = v + w t + q_xyz x t
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def pixel_to_ground(u, v, width, height, hfov, camera, q, ground_z):
    """Where the ray through pixel (u, v) hits the plane z = ground_z.

    The camera looks down from `camera` (x, y, z) on a body with orientation q
    (w, x, y, z). Image up is the body's +x and image left its +y, like the X3
    camera, so a tilted UAV is handled too. None if the ray misses the plane.
    """
    f = (width / 2) / math.tan(hfov / 2)
    ray = rotate(q, ((height / 2 - v) / f, (width / 2 - u) / f, -1.0))
    if ray[2] >= 0:
        return None
    t = (ground_z - camera[2]) / ray[2]
    if t <= 0:
        return None
    return camera[0] + t * ray[0], camera[1] + t * ray[1]
