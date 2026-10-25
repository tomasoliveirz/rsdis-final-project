#!/usr/bin/env python3
"""Builds the Nav2 map from the boxes and cylinders of a Gazebo world.

The map origin is the world origin, so map and Gazebo coordinates match.

    python3 make_map.py worlds/hazard_zone.sdf maps/hazard_zone
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

# only obstacles at the height of the lidar go into the map
LIDAR_MIN_Z = 0.05
LIDAR_MAX_Z = 0.5


def pose_of(elem):
    p = elem.find('pose')
    if p is None or not p.text:
        return 0.0, 0.0, 0.0, 0.0
    v = [float(s) for s in p.text.split()]
    return v[0], v[1], v[2], v[5]


def compose(a, b):
    ax, ay, az, ayaw = a
    bx, by, bz, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return ax + c * bx - s * by, ay + s * bx + c * by, az + bz, ayaw + byaw


def obstacles(world):
    for model in world.iter('model'):
        if (model.findtext('static') or '').strip().lower() not in ('true', '1'):
            continue
        mpose = pose_of(model)
        for link in model.findall('link'):
            lpose = compose(mpose, pose_of(link))
            for col in link.findall('collision'):
                x, y, z, yaw = compose(lpose, pose_of(col))
                geom = col.find('geometry')
                b, c = geom.find('box'), geom.find('cylinder')
                if b is not None:
                    sx, sy, sz = (float(s) for s in b.findtext('size').split())
                    shape = ('box', x, y, yaw, sx, sy)
                elif c is not None:
                    r, sz = float(c.findtext('radius')), float(c.findtext('length'))
                    shape = ('cyl', x, y, 0.0, r, r)
                else:
                    continue  # planes, meshes
                if z + sz / 2 < LIDAR_MIN_Z or z - sz / 2 > LIDAR_MAX_Z:
                    continue
                yield shape


def occupied(shape, px, py):
    kind, x, y, yaw, a, b = shape
    dx, dy = px - x, py - y
    if kind == 'cyl':
        return dx * dx + dy * dy <= a * a
    c, s = math.cos(-yaw), math.sin(-yaw)
    u, v = c * dx - s * dy, s * dx + c * dy
    return abs(u) <= a / 2 and abs(v) <= b / 2


def extents(shape):
    """Half size of the axis-aligned box around the shape."""
    kind, _, _, yaw, a, b = shape
    if kind == 'cyl':
        return a, a
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    return c * a / 2 + s * b / 2, s * a / 2 + c * b / 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world')
    ap.add_argument('out', help='output path without extension')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--margin', type=float, default=1.0)
    args = ap.parse_args()

    world = ET.parse(args.world).getroot().find('world')
    shapes = list(obstacles(world))
    ext = [extents(s) for s in shapes]
    xmin = min(s[1] - e[0] for s, e in zip(shapes, ext)) - args.margin
    xmax = max(s[1] + e[0] for s, e in zip(shapes, ext)) + args.margin
    ymin = min(s[2] - e[1] for s, e in zip(shapes, ext)) - args.margin
    ymax = max(s[2] + e[1] for s, e in zip(shapes, ext)) + args.margin

    res = args.resolution
    w, h = int(math.ceil((xmax - xmin) / res)), int(math.ceil((ymax - ymin) / res))
    rows = []
    for j in range(h):  # PGM rows go from top (ymax) to bottom
        py = ymax - (j + 0.5) * res
        row = bytearray(254 for _ in range(w))
        for s, (ex, ey) in zip(shapes, ext):
            if abs(py - s[2]) > ey:
                continue
            i0 = max(0, int((s[1] - ex - xmin) / res))
            i1 = min(w, int((s[1] + ex - xmin) / res) + 1)
            for i in range(i0, i1):
                if occupied(s, xmin + (i + 0.5) * res, py):
                    row[i] = 0
        rows.append(bytes(row))

    with open(args.out + '.pgm', 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.writelines(rows)
    # origin is the bottom-left corner of the image
    with open(args.out + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(args.out)}.pgm\n'
                'mode: trinary\n'
                f'resolution: {res}\n'
                f'origin: [{xmin:.3f}, {ymax - h * res:.3f}, 0.0]\n'
                'negate: 0\n'
                'occupied_thresh: 0.65\n'
                'free_thresh: 0.25\n')
    print(f'{len(shapes)} obstacles, {w}x{h} cells -> {args.out}.pgm')


if __name__ == '__main__':
    main()
