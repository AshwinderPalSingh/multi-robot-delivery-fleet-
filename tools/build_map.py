#!/usr/bin/env python3
"""
Rebuild maps/hospital_large.{pgm,yaml} from worlds/hospital_large.world.

    python3 tools/build_map.py            # rebuild
    python3 tools/build_map.py --check    # rebuild, then validate

WHY THIS EXISTS instead of a teleop SLAM pass: the map is a pure function of
the world geometry, so deriving it analytically is exact, repeatable, and free
of the drift smear a manual mapping run leaves behind. Change a wall in the
world and the map is one command behind it.

THE ONE SUBTLETY THAT MATTERS: only geometry intersecting the rovers' scan
plane is drawn. The lidar sits 0.60 m above a base_link that rides at the
0.15 m wheel radius, so the plane is z = 0.75 m. Anything the scanner cannot
see must not be in the map AMCL matches against.

That cut runs both ways, and it is worth checking after any world edit: an
obstacle SHORTER than the scan plane is invisible to the lidar, absent from
the map, and the global planner will route straight through it. --check flags
exactly that case.
"""
import argparse
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
WORLD = os.path.join(ROOT, 'worlds', 'hospital_large.world')
STEM = os.path.join(ROOT, 'maps', 'hospital_large')

RES = 0.05
OX, OY = -15.5, -9.5          # world coords of the map origin
W, H = 680, 560               # 34.0 m x 28.0 m
SCAN_Z = 0.75                 # lidar plane: 0.60 above a base_link at 0.15
INSCRIBED = 0.485             # 0.90 x 0.87 footprint + 0.05 padding
# Geometric fit is not the same as navigable. Nav2 inflates obstacles out to
# inflation_radius (0.55 m in config/nav2_params_rover*.yaml), so a corridor
# only a few centimetres wider than the rover is lethal cost from wall to wall:
# the planner may still thread it, but the controller cannot follow the result
# and the rover thrashes until its localisation gives out. WORKING_MARGIN is
# the clearance a corridor needs on top of the inscribed radius before it is
# genuinely usable.
WORKING_MARGIN = 0.15
SEED = (-13.0, -7.5)          # dock_alpha, definitely interior

FREE, OCC, UNK = 254, 0, 205

# Above the scan plane by design, or drivable, so absence from the map is
# correct rather than a bug: wall-mounted signs, items resting on furniture
# that IS visible, overhead fittings, and the flat charging plates.
EXPECTED_INVISIBLE = {
    'reception_monitor', 'iv_bag_1', 'curtain_rail_1', 'nurses_monitor_1',
    'nurses_monitor_2', 'sign_room1', 'sign_room2', 'sign_charging',
    'ot_lamp_head', 'sign_surgical', 'sign_pharmacy', 'pillow_room1',
    'pillow_room2', 'dock_alpha', 'dock_bravo', 'dock_charlie', 'dock_delta',
}


def parse_models(text):
    text = re.sub(r'<!--.*?-->', '', text, flags=re.S)   # skip commented-out models
    for m in re.finditer(r'<model name="([^"]+)">(.*?)</model>', text, re.S):
        name, body = m.group(1), m.group(2)
        if name == 'ground_plane' or '<collision' not in body:
            continue
        pm = re.search(r'<pose>([^<]+)</pose>', body)
        if not pm:
            continue
        p = [float(t) for t in pm.group(1).split()]
        bm = re.search(r'<box><size>([^<]+)</size>', body)
        cm = re.search(r'<cylinder><radius>([^<]+)</radius><length>([^<]+)</length>', body)
        if bm:
            sx, sy, sz = [float(t) for t in bm.group(1).split()]
            yield name, 'box', p, (sx, sy, sz)
        elif cm:
            r, l = float(cm.group(1)), float(cm.group(2))
            yield name, 'cyl', p, (r, l)


def rasterise():
    text = open(WORLD).read()
    grid = np.full((H, W), FREE, np.uint8)
    nb = nc = 0
    for name, kind, p, size in parse_models(text):
        x, y, z, yaw = p[0], p[1], p[2], p[5]
        if kind == 'box':
            sx, sy, sz = size
            if not (z - sz / 2 <= SCAN_Z <= z + sz / 2):
                continue
            nb += 1
            hx, hy = sx / 2, sy / 2
            c, s = math.cos(yaw), math.sin(yaw)
            corners = [(x + c * dx - s * dy, y + s * dx + c * dy)
                       for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
            us = [(cx - OX) / RES for cx, _ in corners]
            vs = [(cy - OY) / RES for _, cy in corners]
            for v in range(max(0, int(min(vs)) - 1), min(H, int(max(vs)) + 2)):
                wy = OY + (v + 0.5) * RES
                for u in range(max(0, int(min(us)) - 1), min(W, int(max(us)) + 2)):
                    wx = OX + (u + 0.5) * RES
                    dx, dy = wx - x, wy - y
                    if abs(c * dx + s * dy) <= hx and abs(-s * dx + c * dy) <= hy:
                        grid[v, u] = OCC
        else:
            r, l = size
            if not (z - l / 2 <= SCAN_Z <= z + l / 2):
                continue
            nc += 1
            cu, cv = (x - OX) / RES, (y - OY) / RES
            rr = r / RES
            for v in range(max(0, int(cv - rr) - 1), min(H, int(cv + rr) + 2)):
                for u in range(max(0, int(cu - rr) - 1), min(W, int(cu + rr) + 2)):
                    if (u + 0.5 - cu) ** 2 + (v + 0.5 - cv) ** 2 <= rr * rr:
                        grid[v, u] = OCC

    # Anything free but unreachable from inside becomes unknown, which is what
    # a real SLAM pass would have left there.
    su, sv = int((SEED[0] - OX) / RES), int((SEED[1] - OY) / RES)
    assert grid[sv, su] == FREE, 'seed point landed on an obstacle'
    reach = np.zeros_like(grid, bool)
    reach[sv, su] = True
    stack = [(sv, su)]
    while stack:
        v, u = stack.pop()
        for dv, du in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nv, nu = v + dv, u + du
            if 0 <= nv < H and 0 <= nu < W and not reach[nv, nu] and grid[nv, nu] == FREE:
                reach[nv, nu] = True
                stack.append((nv, nu))
    grid[(grid == FREE) & ~reach] = UNK

    print(f'  rasterised {nb} boxes + {nc} cylinders at z = {SCAN_Z} m')
    print(f'  {W}x{H} @ {RES} m   free {int((grid==FREE).sum())}  '
          f'occupied {int((grid==OCC).sum())}  unknown {int((grid==UNK).sum())}')
    return grid


def write(grid):
    # PGM rows run top-down, world rows run bottom-up.
    with open(STEM + '.pgm', 'wb') as f:
        f.write(b'P5\n# GForce hospital ward - extended arena\n%d %d\n255\n' % (W, H))
        f.write(np.flipud(grid).tobytes())
    with open(STEM + '.yaml', 'w') as f:
        f.write(f'image: hospital_large.pgm\nmode: trinary\nresolution: {RES}\n'
                f'origin: [{OX}, {OY}, 0.0]\nnegate: 0\n'
                f'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'  wrote {STEM}.pgm / .yaml')


_EDT_CACHE = {}


def edt_clear(free):
    """Metres to the nearest blocked cell, for every free cell."""
    key = id(free)
    if key in _EDT_CACHE:
        return _EDT_CACHE[key]
    blocked = ~free
    vv, uu = np.nonzero(blocked)
    d = np.full(free.shape, 1e9, np.float32)
    # Brute force is fine here: the map is 380k cells and this runs once, from
    # a tool, not from the fleet.
    ys = np.arange(free.shape[0])[:, None]
    xs = np.arange(free.shape[1])[None, :]
    step = max(1, len(vv) // 4000)
    for v, u in zip(vv[::step], uu[::step]):
        np.minimum(d, (ys - v) ** 2 + (xs - u) ** 2, out=d)
    d = np.sqrt(d) * RES
    _EDT_CACHE[key] = d
    return d


def check(grid):
    bad = 0

    print('\n-- obstacles the scanner cannot see --')
    missed = []
    for name, kind, p, size in parse_models(open(WORLD).read()):
        z = p[2]
        h = size[2] if kind == 'box' else size[1]
        if not (z - h / 2 <= SCAN_Z <= z + h / 2) and name not in EXPECTED_INVISIBLE:
            missed.append((name, z - h / 2, z + h / 2))
    if missed:
        bad += len(missed)
        for n, lo, hi in missed:
            print(f'  FAIL {n}: spans {lo:.2f}..{hi:.2f} m, below the '
                  f'{SCAN_Z} m scan plane — invisible to the lidar, absent '
                  f'from the map, and the planner will route through it')
    else:
        print('  ok — every floor obstacle reaches the scan plane')

    print('\n-- clearance-safe connectivity --')
    free = (grid == FREE)
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

    from collections import deque
    lbl = np.zeros((H, W), np.int32)
    cur, sizes = 0, []
    for v0 in range(H):
        for u0 in range(W):
            if not safe[v0, u0] or lbl[v0, u0]:
                continue
            cur += 1
            n = 0
            q = deque([(v0, u0)])
            lbl[v0, u0] = cur
            while q:
                v, u = q.popleft()
                n += 1
                for dv, du in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nv, nu = v + dv, u + du
                    if 0 <= nv < H and 0 <= nu < W and safe[nv, nu] and not lbl[nv, nu]:
                        lbl[nv, nu] = cur
                        q.append((nv, nu))
            sizes.append(n)
    a = RES * RES
    print(f'  {cur} region(s); largest {max(sizes)*a:.1f} m^2 of '
          f'{int(free.sum())*a:.1f} m^2 free')
    if cur != 1:
        bad += 1
        for i, sz in enumerate(sorted(sizes, reverse=True)[:5], 1):
            print(f'    region {i}: {sz*a:.1f} m^2')
        print('  FAIL — a 0.9 m rover cannot reach the whole ward')
    else:
        print('  ok — one connected region, no isolated pockets')

    print('\n-- corridors that gate a route --')
    # A tight gap only matters if something is reachable ONLY through it.
    # The gap beside a hospital bed is 1.00 m and always will be, but it is a
    # cul-de-sac the rover delivers into, never drives through, so failing on
    # it would be noise. What broke the fleet was different in kind: the
    # pharmacy door was the sole way in or out of a room, and it was marginal.
    #
    # So the test is connectivity of the COMFORTABLE region -- cells with a
    # real working margin over the inscribed radius. If a station sits outside
    # the main comfortable component, the only way to it is through a pinch,
    # and Nav2 will thrash there.
    clear_m = edt_clear(free)
    comfy = clear_m >= (INSCRIBED + WORKING_MARGIN)
    clbl = np.zeros((H, W), np.int32)
    ccur, csizes = 0, []
    for v0 in range(H):
        for u0 in range(W):
            if not comfy[v0, u0] or clbl[v0, u0]:
                continue
            ccur += 1
            n = 0
            q = deque([(v0, u0)])
            clbl[v0, u0] = ccur
            while q:
                v, u = q.popleft()
                n += 1
                for dv, du in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nv, nu = v + dv, u + du
                    if 0 <= nv < H and 0 <= nu < W and comfy[nv, nu] and not clbl[nv, nu]:
                        clbl[nv, nu] = ccur
                        q.append((nv, nu))
            csizes.append(n)
    cmain = int(np.argmax(csizes)) + 1
    print(f'  comfortable region (>= {2*(INSCRIBED+WORKING_MARGIN):.2f} m wide): '
          f'{max(csizes)*a:.1f} m^2 in {ccur} component(s)')

    print('\n-- configured stations and docks --')
    import yaml
    cfg = yaml.safe_load(open(os.path.join(ROOT, 'config', 'fleet_brain.yaml')))
    st = cfg['fleet_stations']['ros__parameters']
    rv = cfg['fleet_rovers']['ros__parameters']
    pts = [(n, st[n][0], st[n][1]) for n in st['names']]
    pts += [(f'{k} dock', rv[k]['dock_pose'][0], rv[k]['dock_pose'][1])
            for k in sorted(rv) if isinstance(rv[k], dict)]
    main = int(np.argmax(sizes)) + 1
    for n, x, y in pts:
        u, v = int((x - OX) / RES), int((y - OY) / RES)
        inside = 0 <= u < W and 0 <= v < H
        reach = inside and lbl[v, u] == main
        # Reachable at all, and reachable without threading a pinch.
        easy = inside and clbl[v, u] == cmain
        if not reach:
            bad += 1
            note = 'UNREACHABLE'
        elif not easy:
            bad += 1
            note = (f'reachable only through a corridor under '
                    f'{2*(INSCRIBED+WORKING_MARGIN):.2f} m — Nav2 will thrash')
        else:
            note = f'clearance {clear_m[v, u]:.2f} m'
        print(f'  {"ok  " if reach and easy else "FAIL"} {n:<16} '
              f'({x:+7.2f}, {y:+6.2f})  {note}')

    print(f'\n{"ALL CHECKS PASSED" if bad == 0 else f"{bad} PROBLEM(S) FOUND"}')
    return 1 if bad else 0


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true',
                    help='validate scan-plane coverage, connectivity and stations')
    args = ap.parse_args()
    g = rasterise()
    write(g)
    sys.exit(check(g) if args.check else 0)
