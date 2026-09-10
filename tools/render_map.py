#!/usr/bin/env python3
"""
Render maps/hospital_large_annotated.png — the human-readable ward plan.

    python3 tools/render_map.py

Shows what the occupancy grid alone does not: which floor a 0.9 m rover can
actually stand on, where the delivery stations are and how much demand each
carries (the weights factor F7 integrates over), and where each rover lives.
Stations and docks are read from config/fleet_brain.yaml, so the picture cannot
drift away from what the brain is actually configured with.
"""
import os

import numpy as np
import yaml
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
STEM = os.path.join(ROOT, 'maps', 'hospital_large')
RES = 0.05
OX, OY = -15.5, -9.5
INSCRIBED = 0.485
SCALE = 2
TOP, GUTTER = 64, 170

ROVER_COLOUR = {'rover1': (30, 170, 60), 'rover2': (230, 120, 0),
                'rover3': (0, 150, 200), 'rover4': (200, 0, 150)}


def load_pgm():
    raw = open(STEM + '.pgm', 'rb').read()
    i = 0
    for _ in range(4):
        i = raw.index(b'\n', i) + 1
    w, h = [int(t) for t in raw[:i].decode().split('\n')[2].split()]
    g = np.flipud(np.frombuffer(raw[i:], np.uint8).reshape(h, w)).copy()
    return g, w, h


def main():
    g, W, H = load_pgm()
    free = (g == 254)

    # Erode by the inscribed radius: where the rover's whole footprint fits.
    r = INSCRIBED / RES
    R = int(np.ceil(r))
    safe = free.copy()
    for dv in range(-R, R + 1):
        for du in range(-R, R + 1):
            if dv * dv + du * du > r * r:
                continue
            sh = np.zeros_like(free)
            v0, v1 = max(0, dv), H + min(0, dv)
            u0, u1 = max(0, du), W + min(0, du)
            sh[v0:v1, u0:u1] = free[v0 - dv:v1 - dv, u0 - du:u1 - du]
            safe &= sh

    rgb = np.full((H, W, 3), 24, np.uint8)
    rgb[g == 205] = (58, 60, 68)
    rgb[free] = (238, 240, 244)
    rgb[safe] = (198, 228, 206)
    rgb[g == 0] = (38, 42, 52)

    base = Image.fromarray(np.flipud(rgb)).convert('RGB') \
                .resize((W * SCALE, H * SCALE), Image.NEAREST)
    im = Image.new('RGB', (W * SCALE + GUTTER, H * SCALE + TOP), (250, 250, 252))
    im.paste(base, (0, TOP))
    d = ImageDraw.Draw(im)

    def px(x, y):
        return ((x - OX) / RES * SCALE, TOP + (H - (y - OY) / RES) * SCALE)

    cfg = yaml.safe_load(open(os.path.join(ROOT, 'config', 'fleet_brain.yaml')))
    st = cfg['fleet_stations']['ros__parameters']
    rv = cfg['fleet_rovers']['ros__parameters']
    right = W * SCALE + GUTTER

    for n in st['names']:
        x, y, _yaw, dem = st[n]
        cx, cy = px(x, y)
        rr = 6 + dem * 7
        d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr],
                  fill=(40, 120, 200), outline=(255, 255, 255), width=2)
        d.text((min(cx + rr + 5, right - 95), cy - 6), n, fill=(15, 25, 45))

    for k in sorted(ROVER_COLOUR):
        x, y, _ = rv[k]['dock_pose']
        cx, cy = px(x, y)
        d.rectangle([cx - 11, cy - 11, cx + 11, cy + 11],
                    fill=ROVER_COLOUR[k], outline=(255, 255, 255), width=2)
        d.text((min(cx + 16, right - 150), cy - 6),
               f"{k} - {rv[k]['dock']}", fill=(15, 25, 45))

    area = int(safe.sum()) * RES * RES
    d.text((16, 12), 'GForce ward  -  33.2 x 27.2 m  -  4 rovers, 1 central brain',
           fill=(15, 25, 45))
    d.text((16, 30), f'green = a 0.9 m rover fits ({area:.0f} m2)',
           fill=(55, 70, 95))
    d.text((16, 45), 'circles = delivery stations, sized by demand weight '
                     '(what factor F7 integrates over)   |   '
                     'squares = charging docks / rover home poses',
           fill=(55, 70, 95))

    out = STEM + '_annotated.png'
    im.save(out)
    print(f'wrote {out}  {im.size[0]}x{im.size[1]}')


if __name__ == '__main__':
    main()
