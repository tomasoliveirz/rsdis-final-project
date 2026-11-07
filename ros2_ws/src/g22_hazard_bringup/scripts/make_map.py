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


def map_bounds(shapes, margin):
    sizes = [extents(shape) for shape in shapes]
    xmin = min(shape[1] - size[0] for shape, size in zip(shapes, sizes)) - margin
    xmax = max(shape[1] + size[0] for shape, size in zip(shapes, sizes)) + margin
    ymin = min(shape[2] - size[1] for shape, size in zip(shapes, sizes)) - margin
    ymax = max(shape[2] + size[1] for shape, size in zip(shapes, sizes)) + margin
    return xmin, xmax, ymin, ymax


def rasterize(shapes, bounds, resolution):
    xmin, xmax, ymin, ymax = bounds
    width = int(math.ceil((xmax - xmin) / resolution))
    height = int(math.ceil((ymax - ymin) / resolution))
    sizes = [extents(shape) for shape in shapes]
    rows = []
    for j in range(height):  # PGM starts at the top of the map.
        py = ymax - (j + 0.5) * resolution
        row = bytearray(254 for _ in range(width))
        for shape, (ex, ey) in zip(shapes, sizes):
            if abs(py - shape[2]) > ey:
                continue
            first = max(0, int((shape[1] - ex - xmin) / resolution))
            last = min(width, int((shape[1] + ex - xmin) / resolution) + 1)
            for i in range(first, last):
                if occupied(shape, xmin + (i + 0.5) * resolution, py):
                    row[i] = 0
        rows.append(bytes(row))
    return width, height, rows


def write_image(path, width, height, rows):
    with open(path, 'wb') as file:
        file.write(f'P5\n{width} {height}\n255\n'.encode())
        file.writelines(rows)


def write_metadata(out, resolution, origin):
    x, y = origin
    with open(out + '.yaml', 'w') as file:
        file.write(f'image: {os.path.basename(out)}.pgm\n'
                   'mode: trinary\n'
                   f'resolution: {resolution}\n'
                   f'origin: [{x:.3f}, {y:.3f}, 0.0]\n'
                   'negate: 0\n'
                   'occupied_thresh: 0.65\n'
                   'free_thresh: 0.25\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('world')
    parser.add_argument('out', help='output path without extension')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--margin', type=float, default=1.0)
    args = parser.parse_args()
    world = ET.parse(args.world).getroot().find('world')
    shapes = list(obstacles(world))
    bounds = map_bounds(shapes, args.margin)
    width, height, rows = rasterize(shapes, bounds, args.resolution)
    write_image(args.out + '.pgm', width, height, rows)
    write_metadata(args.out, args.resolution, (bounds[0], bounds[3] - height * args.resolution))
    print(f'{len(shapes)} obstacles, {width}x{height} cells -> {args.out}.pgm')


if __name__ == '__main__':
    main()
