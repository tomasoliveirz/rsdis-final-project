#!/usr/bin/env python3
"""Makes the signed barrels: textures for the lid and the side, and the two meshes
(an open tube and a disc) with texture coordinates that fit them.

Radiation: trefoil with the standard proportions (centre radius r, blades from
1.5r to 5r, 60 degrees each). Gas: GHS02 flammable pictogram, black flame in a
red diamond. There are also three look-alikes that are not hazard signs: a black
ring on dull yellow, an all-black diamond and an empty red diamond.

    python3 make_barrels.py models/barrel
"""
from functools import partial
import math
import os
import sys

import cv2
import numpy as np

YELLOW = (0, 210, 255)  # BGR
RED = (0, 0, 210)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RADIUS, HEIGHT, SEGMENTS = 0.3, 0.9, 48
# barrel colours, same as in worlds/hazard_zone.sdf
STEEL = (125, 117, 110)
TOXIC_GREEN = (55, 135, 100)
MUSTARD = (95, 165, 205)
GREY = (150, 150, 145)


def trefoil(img, cx, cy, r):
    for angle in (90, 210, 330):
        cv2.ellipse(img, (cx, cy), (5 * r, 5 * r), 0, angle - 30, angle + 30, BLACK, -1, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), int(1.5 * r), YELLOW, -1, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), r, BLACK, -1, cv2.LINE_AA)


def flame(img, cx, cy, s):
    """GHS-style flame centred on (cx, cy), about 2s tall."""
    def drop(w, h, x0, y0, n=60):
        # teardrop: round bottom, pointed top
        pts = [(x0 + w * math.sin(t) * math.sin(t / 2) ** 1.5, y0 - h * math.cos(t))
               for t in np.linspace(0, 2 * math.pi, n)]
        return np.array(pts, np.int32)
    cv2.fillPoly(img, [drop(0.62 * s, s, cx, cy)], BLACK, cv2.LINE_AA)
    cv2.fillPoly(img, [drop(0.28 * s, 0.55 * s, cx - 0.45 * s, cy + 0.32 * s)], BLACK, cv2.LINE_AA)
    cv2.fillPoly(img, [drop(0.22 * s, 0.4 * s, cx + 0.05 * s, cy + 0.5 * s)], WHITE, cv2.LINE_AA)
    base = int(cy + 1.08 * s)
    cv2.rectangle(img, (int(cx - 0.75 * s), base), (int(cx + 0.75 * s), base + max(2, int(0.1 * s))),
                  BLACK, -1)


def radiation_sign(img, cx, cy, size):
    cv2.circle(img, (cx, cy), size // 2, YELLOW, -1, cv2.LINE_AA)
    trefoil(img, cx, cy, int(size * 0.085))


def gas_sign(img, cx, cy, size, outline=RED, with_flame=True):
    h = size // 2
    diamond = np.array([[cx, cy - h], [cx + h, cy], [cx, cy + h], [cx - h, cy]], np.int32)
    cv2.fillPoly(img, [diamond], WHITE, cv2.LINE_AA)
    cv2.polylines(img, [diamond], True, outline, max(2, size // 14), cv2.LINE_AA)
    if with_flame:
        flame(img, cx, cy - int(0.04 * size), int(0.26 * size))


def ring_sign(img, cx, cy, size):
    cv2.circle(img, (cx, cy), int(size * 0.4), BLACK, max(2, size // 12), cv2.LINE_AA)
    cv2.circle(img, (cx, cy), int(size * 0.07), BLACK, -1, cv2.LINE_AA)


# kind: lid background, barrel colour, how to draw the sign, sign size on the lid
KINDS = {
    'radiation': (YELLOW, STEEL, radiation_sign, 0.98),
    'gas': (WHITE, TOXIC_GREEN, gas_sign, 0.68),
    'ring': (MUSTARD, MUSTARD, ring_sign, 0.9),
    'black_diamond': (WHITE, GREY, partial(gas_sign, outline=BLACK), 0.68),
    'empty_diamond': (WHITE, GREY, partial(gas_sign, with_flame=False), 0.68),
}


def lid(kind, n=512):
    """Square texture for the barrel lid (the cylinder cap shows the inscribed circle)."""
    background, _, draw, size = KINDS[kind]
    img = np.full((n, n, 3), background, np.uint8)
    draw(img, n // 2, n // 2, int(n * size))
    return img


def side(kind, w=1024, h=512):
    """Label wrapped around the barrel side: barrel colour with three signs."""
    _, colour, draw, _ = KINDS[kind]
    img = np.full((h, w, 3), colour, np.uint8)
    for k in range(3):
        draw(img, int(w * (k + 0.5) / 3), h // 2, int(h * 0.55))
    return img


def write_side(path):
    """Open tube along z, centred on the origin. u goes around, v goes up."""
    lines = []
    for i in range(SEGMENTS + 1):
        a = 2 * math.pi * i / SEGMENTS
        x, y = RADIUS * math.cos(a), RADIUS * math.sin(a)
        lines += [f'v {x:.5f} {y:.5f} {-HEIGHT / 2}', f'v {x:.5f} {y:.5f} {HEIGHT / 2}']
        lines += [f'vt {i / SEGMENTS:.5f} 0', f'vt {i / SEGMENTS:.5f} 1']
        lines += [f'vn {math.cos(a):.5f} {math.sin(a):.5f} 0'] * 2
    for i in range(SEGMENTS):
        b0, t0, b1, t1 = 2 * i + 1, 2 * i + 2, 2 * i + 3, 2 * i + 4
        lines += [f'f {b0}/{b0}/{b0} {b1}/{b1}/{b1} {t1}/{t1}/{t1}',
                  f'f {b0}/{b0}/{b0} {t1}/{t1}/{t1} {t0}/{t0}/{t0}']
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def write_lid(path):
    """Disc facing up at z = 0, with the texture mapped flat on it."""
    lines = ['v 0 0 0', 'vt 0.5 0.5', 'vn 0 0 1']
    for i in range(SEGMENTS + 1):
        a = 2 * math.pi * i / SEGMENTS
        lines += [f'v {RADIUS * math.cos(a):.5f} {RADIUS * math.sin(a):.5f} 0',
                  f'vt {0.5 + 0.5 * math.cos(a):.5f} {0.5 + 0.5 * math.sin(a):.5f}', 'vn 0 0 1']
    for i in range(SEGMENTS):
        a, b = i + 2, i + 3
        lines.append(f'f 1/1/1 {a}/{a}/{a} {b}/{b}/{b}')
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    for kind in KINDS:
        cv2.imwrite(os.path.join(out, f'{kind}_lid.png'), lid(kind))
        cv2.imwrite(os.path.join(out, f'{kind}_side.png'), side(kind))
    write_side(os.path.join(out, 'barrel_side.obj'))
    write_lid(os.path.join(out, 'barrel_lid.obj'))
