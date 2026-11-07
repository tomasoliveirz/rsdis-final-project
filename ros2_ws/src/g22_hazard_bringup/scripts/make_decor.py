#!/usr/bin/env python3
"""Builds the textures and meshes of the industrial decor (models/g22_decor).

Everything is generated here (procedural textures, simple OBJ meshes), so we
own it and can rebuild it at any time:

    python3 make_decor.py ../models/g22_decor

The meshes have the same size as the boxes and cylinders of the world, so the
collisions (and the Nav2 map made by make_map.py) do not change. Only the
visuals of the world use them.

Needs numpy and Pillow (python3-numpy, python3-pil).
"""
import argparse
import math
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

RNG = np.random.default_rng(22)  # group 22, fixed seed so the output is the same every time



def noise(h, w, beta=2.0):
    """Tileable fractal noise in [0, 1] (random phases filtered as 1/f^beta)."""
    fy = np.fft.fftfreq(h)[:, None]
    fx = np.fft.fftfreq(w)[None, :]
    f = np.sqrt(fx ** 2 + fy ** 2)
    f[0, 0] = 1.0
    spectrum = np.fft.fft2(RNG.standard_normal((h, w))) / f ** (beta / 2)
    spectrum[0, 0] = 0
    n = np.real(np.fft.ifft2(spectrum))
    return (n - n.min()) / (n.max() - n.min())


def mix(a, b, t):
    """Blend two RGB colours (0-1) with weight t (h x w)."""
    a, b = np.array(a)[None, None, :], np.array(b)[None, None, :]
    return a * (1 - t[..., None]) + b * t[..., None]


def save(rgb, path):
    Image.fromarray((np.clip(rgb, 0, 1) * 255).astype(np.uint8)).save(path)


def rust_mask(h, w, amount, bottom_bias=0.0):
    """Patches where the paint is gone. bottom_bias puts more rust near the bottom."""
    n = noise(h, w, 2.4) * 0.7 + noise(h, w, 1.2) * 0.3
    if bottom_bias:
        n = n + bottom_bias * np.linspace(0, 1, h)[:, None] ** 2
    t = np.quantile(n, 1 - amount)
    return np.clip((n - t) * 12, 0, 1)


def streaks(h, w, count, length):
    """Dark streaks running down from random points (rain marks, leaks)."""
    img = Image.new('L', (w, h), 0)
    d = ImageDraw.Draw(img)
    for _ in range(count):
        x = RNG.uniform(0, w)
        y = RNG.uniform(0, h * 0.8)
        width = RNG.uniform(2, 10)
        d.line([(x, y), (x + RNG.uniform(-4, 4), y + RNG.uniform(0.3, 1) * length)],
               fill=int(RNG.uniform(60, 160)), width=int(width))
    img = img.filter(ImageFilter.GaussianBlur(3))
    return np.asarray(img) / 255.0



def tex_dirt(size=1024):
    """Packed dirt with darker damp patches and gravel. Tileable."""
    big = noise(size, size, 2.2)
    fine = noise(size, size, 0.6)
    rgb = mix((0.47, 0.39, 0.29), (0.30, 0.24, 0.17), np.clip(big * 1.4 - 0.2, 0, 1))
    rgb *= (0.85 + 0.3 * fine)[..., None]
    img = Image.fromarray((np.clip(rgb, 0, 1) * 255).astype(np.uint8))
    d = ImageDraw.Draw(img)
    for _ in range(2500):  # small stones, drawn 3x3 times so the texture still tiles
        x, y, r = RNG.uniform(0, size), RNG.uniform(0, size), RNG.uniform(1.5, 5)
        g = int(RNG.uniform(70, 170))
        col = (g, int(g * 0.95), int(g * 0.88))
        for ox in (-size, 0, size):
            for oy in (-size, 0, size):
                d.ellipse([x + ox - r, y + oy - r * 0.8, x + ox + r, y + oy + r * 0.8], fill=col)
    img = img.filter(ImageFilter.GaussianBlur(0.8))
    return np.asarray(img) / 255.0


def crack_mask(size):
    mask = Image.new('L', (size, size), 0)
    draw = ImageDraw.Draw(mask)
    for _ in range(6):
        x, y = RNG.uniform(0, size), RNG.uniform(0, size)
        angle = RNG.uniform(0, 2 * math.pi)
        pts = [(x, y)]
        for _ in range(25):  # a crack wanders slowly in one direction
            angle += RNG.uniform(-0.6, 0.6)
            x += 12 * math.cos(angle)
            y += 12 * math.sin(angle)
            pts.append((x, y))
        draw.line(pts, fill=255, width=2)
    return mask


def tex_concrete(size=1024, cracks=False):
    """One 2.5 m concrete panel: stains, a joint on the edges and tie holes."""
    n = noise(size, size, 1.8)
    rgb = mix((0.62, 0.61, 0.58), (0.45, 0.44, 0.41), n)
    rgb *= (0.9 + 0.2 * noise(size, size, 0.5))[..., None]
    stain = streaks(size, size, 25, size * 0.5) + 0.6 * np.linspace(0, 1, size)[:, None] ** 3
    rgb *= (1 - 0.35 * np.clip(stain, 0, 1))[..., None]
    rgb[:, :6] *= 0.45
    rgb[:, -6:] *= 0.45
    img = Image.fromarray((np.clip(rgb, 0, 1) * 255).astype(np.uint8))
    d = ImageDraw.Draw(img)
    for fx in (0.2, 0.8):
        for fy in (0.25, 0.75):
            x, y = fx * size, fy * size
            d.ellipse([x - 9, y - 9, x + 9, y + 9], fill=(70, 68, 64))
    out = np.asarray(img) / 255.0
    if cracks:
        m = np.asarray(crack_mask(size).filter(ImageFilter.GaussianBlur(0.7))) / 255.0
        out = out * (1 - 0.7 * m[..., None])
    return out


def tex_corrugated(colour, w=1024, h=512, period=26, rust=0.18, frame=True):
    """Corrugated steel (vertical ribs) for the shipping containers and the shed."""
    x = np.arange(w)[None, :]
    ribs = 0.82 + 0.18 * np.sin(2 * np.pi * x / period)
    rgb = np.ones((h, w, 3)) * np.array(colour)[None, None, :] * ribs[..., None]
    rgb *= (0.85 + 0.3 * noise(h, w, 1.5))[..., None]
    r = rust_mask(h, w, rust, bottom_bias=0.25)
    rust_col = mix((0.45, 0.22, 0.10), (0.25, 0.12, 0.06), noise(h, w, 0.8))
    rgb = rgb * (1 - r[..., None]) + rust_col * r[..., None]
    rgb *= (1 - 0.3 * streaks(h, w, 30, h * 0.6))[..., None]
    if frame:
        b = h // 20
        rgb[:b] *= 0.55
        rgb[-b:] *= 0.55
        rgb[:, :b] *= 0.6
        rgb[:, -b:] *= 0.6
    return rgb


def tex_tank(w=1024, h=512):
    """Storage tank: off-white plates, weld seams, rust leaks and a hazard band."""
    rgb = mix((0.80, 0.79, 0.74), (0.62, 0.60, 0.55), noise(h, w, 1.6))
    for k in range(1, 4):
        rgb[int(k * h / 4) - 2:int(k * h / 4) + 2] *= 0.7  # weld seams
    rust = streaks(h, w, 60, h * 0.7)
    rgb = rgb * (1 - 0.6 * rust[..., None]) + np.array([0.35, 0.17, 0.07]) * 0.6 * rust[..., None]
    y0, y1 = int(0.12 * h), int(0.2 * h)  # yellow / black diagonal stripes near the top
    yy, xx = np.mgrid[y0:y1, 0:w]
    stripes = ((xx + yy) // 40) % 2 == 0
    rgb[y0:y1] = np.where(stripes[..., None], [0.85, 0.68, 0.08], [0.08, 0.08, 0.08])
    return rgb


def tex_rusty_metal(size=512):
    """Old painted steel for the pipe rack."""
    rgb = mix((0.42, 0.40, 0.36), (0.30, 0.29, 0.27), noise(size, size, 1.2))
    r = rust_mask(size, size, 0.45)
    rust_col = mix((0.50, 0.25, 0.10), (0.28, 0.13, 0.05), noise(size, size, 0.7))
    return rgb * (1 - r[..., None]) + rust_col * r[..., None]



class Obj:
    """Minimal OBJ writer (positions, texture coordinates, normals, quads)."""

    def __init__(self):
        self.v, self.vt, self.vn, self.f = [], [], [], []

    def quad(self, corners, uvs, normal):
        base_v, base_t = len(self.v) + 1, len(self.vt) + 1
        self.v += corners
        self.vt += uvs
        self.vn.append(normal)
        n = len(self.vn)
        self.f.append([(base_v + i, base_t + i, n) for i in range(4)])

    def write(self, path, material):
        name = os.path.splitext(os.path.basename(path))[0]
        with open(path, 'w') as f:
            f.write(f'# {name}, made by make_decor.py\nmtllib {material}.mtl\nusemtl {material}\n')
            f.writelines(f'v {x:.4f} {y:.4f} {z:.4f}\n' for x, y, z in self.v)
            f.writelines(f'vt {u:.4f} {v:.4f}\n' for u, v in self.vt)
            f.writelines(f'vn {x:.4f} {y:.4f} {z:.4f}\n' for x, y, z in self.vn)
            for face in self.f:
                f.write('f ' + ' '.join(f'{a}/{b}/{c}' for a, b, c in face) + '\n')


def box(sx, sy, sz, tile=None):
    """Box centred on the origin. tile=(u, v) metres per texture repeat,
    None stretches the texture once over each face."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    m = Obj()

    def uv(a, b):
        if tile is None:
            return [(0, 0), (1, 0), (1, 1), (0, 1)]
        u, v = a / tile[0], b / tile[1]
        return [(0, 0), (u, 0), (u, v), (0, v)]

    m.quad([(-hx, -hy, -hz), (hx, -hy, -hz), (hx, -hy, hz), (-hx, -hy, hz)], uv(sx, sz), (0, -1, 0))
    m.quad([(hx, hy, -hz), (-hx, hy, -hz), (-hx, hy, hz), (hx, hy, hz)], uv(sx, sz), (0, 1, 0))
    m.quad([(hx, -hy, -hz), (hx, hy, -hz), (hx, hy, hz), (hx, -hy, hz)], uv(sy, sz), (1, 0, 0))
    m.quad([(-hx, hy, -hz), (-hx, -hy, -hz), (-hx, -hy, hz), (-hx, hy, hz)], uv(sy, sz), (-1, 0, 0))
    m.quad([(-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)], uv(sx, sy), (0, 0, 1))
    m.quad([(-hx, hy, -hz), (hx, hy, -hz), (hx, -hy, -hz), (-hx, -hy, -hz)], uv(sx, sy), (0, 0, -1))
    return m


def cylinder(r, length, segments=40):
    """Cylinder along z, centred on the origin, texture wrapped once around it."""
    m = Obj()
    h = length / 2
    for i in range(segments):
        a0, a1 = 2 * math.pi * i / segments, 2 * math.pi * (i + 1) / segments
        c0, s0, c1, s1 = math.cos(a0), math.sin(a0), math.cos(a1), math.sin(a1)
        am = (a0 + a1) / 2
        u0, u1 = i / segments, (i + 1) / segments
        m.quad([(r * c0, r * s0, -h), (r * c1, r * s1, -h), (r * c1, r * s1, h), (r * c0, r * s0, h)],
               [(u0, 0), (u1, 0), (u1, 1), (u0, 1)], (math.cos(am), math.sin(am), 0))
        for z, nz in ((h, 1), (-h, -1)):  # caps as thin wedges (quad with two equal corners)
            p0, p1 = (r * c0, r * s0, z), (r * c1, r * s1, z)
            t0 = (0.5 + 0.1 * c0, 0.05 + 0.04 * s0)
            t1 = (0.5 + 0.1 * c1, 0.05 + 0.04 * s1)
            corners = [(0, 0, z), p0, p1, (0, 0, z)] if nz > 0 else [(0, 0, z), p1, p0, (0, 0, z)]
            uvs = [(0.5, 0.05), t0, t1, (0.5, 0.05)] if nz > 0 else [(0.5, 0.05), t1, t0, (0.5, 0.05)]
            m.quad(corners, uvs, (0, 0, nz))
    return m


def ground(size, tile):
    m = Obj()
    h, u = size / 2, size / tile
    m.quad([(-h, -h, 0), (h, -h, 0), (h, h, 0), (-h, h, 0)], [(0, 0), (u, 0), (u, u), (0, u)], (0, 0, 1))
    return m



def decor_textures():
    return {
        'dirt': tex_dirt(),
        'concrete': tex_concrete(),
        'broken_concrete': tex_concrete(512, cracks=True),
        'container_red': tex_corrugated((0.55, 0.16, 0.12)),
        'container_blue': tex_corrugated((0.16, 0.28, 0.45)),
        'container_green': tex_corrugated((0.22, 0.36, 0.22)),
        'container_orange': tex_corrugated((0.70, 0.38, 0.12), rust=0.3),
        'shed': tex_corrugated((0.55, 0.57, 0.58), period=40, rust=0.2, frame=False),
        'tank': tex_tank(),
        'rusty_metal': tex_rusty_metal(),
    }


def decor_meshes():
    # name: (mesh, texture). Sizes are the ones of hazard_zone.sdf.
    meshes = {
        'ground': (ground(120, 4), 'dirt'),
        'wall_ns': (box(30.4, 0.4, 2.5, tile=(2.5, 2.5)), 'concrete'),
        'wall_ew': (box(0.4, 24, 2.5, tile=(2.5, 2.5)), 'concrete'),
        'warehouse': (box(8, 5, 5, tile=(4, 5)), 'shed'),
        'pipe_rack': (box(6, 0.4, 1.2, tile=(1.5, 1.2)), 'rusty_metal'),
        'tank_1': (cylinder(1.6, 5), 'tank'),
        'tank_2': (cylinder(1.2, 4), 'tank'),
        'rubble_1': (box(1.2, 0.8, 0.6, tile=(1, 1)), 'broken_concrete'),
        'rubble_2': (box(0.8, 0.8, 0.5, tile=(1, 1)), 'broken_concrete'),
        'rubble_3': (box(1.5, 0.6, 0.4, tile=(1, 1)), 'broken_concrete'),
        'rubble_4': (box(1, 1, 0.7, tile=(1, 1)), 'broken_concrete'),
        'rubble_5': (box(1.2, 0.5, 0.5, tile=(1, 1)), 'broken_concrete'),
        'rubble_6': (box(0.9, 0.7, 0.5, tile=(1, 1)), 'broken_concrete'),
    }
    for colour in ('red', 'blue', 'green', 'orange'):
        meshes['container_' + colour] = (box(6.1, 2.4, 2.6), 'container_' + colour)
    return meshes


def write_materials(mesh_dir, meshes):
    materials = {texture for _, texture in meshes.values()}
    for name in sorted(materials):
        with open(os.path.join(mesh_dir, name + '.mtl'), 'w') as file:
            file.write(f'newmtl {name}\nKa 1 1 1\nKd 1 1 1\nKs 0.05 0.05 0.05\nNs 10\n'
                       f'map_Kd {name}.png\n')


def write_textures(mesh_dir, textures):
    for name, rgb in textures.items():
        save(rgb, os.path.join(mesh_dir, name + '.png'))


def write_meshes(mesh_dir, meshes):
    for name, (mesh, material) in meshes.items():
        mesh.write(os.path.join(mesh_dir, name + '.obj'), material)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('out', help='model folder, e.g. ../models/g22_decor')
    args = parser.parse_args()
    mesh_dir = os.path.join(args.out, 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)
    textures = decor_textures()
    meshes = decor_meshes()
    write_textures(mesh_dir, textures)
    write_materials(mesh_dir, meshes)
    write_meshes(mesh_dir, meshes)
    print(f'{len(textures)} textures and {len(meshes)} meshes in {mesh_dir}')


if __name__ == '__main__':
    main()
