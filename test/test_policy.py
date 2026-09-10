#!/usr/bin/env python3
"""Offline exercise of the seven-factor assignment policy.

Runs the real scoring code against synthetic fleet states, no Gazebo and no
Nav2. Each case is built so that exactly one factor should decide it, which is
what makes a regression here readable: if the wrong rover wins, the factor
table printed underneath says which term moved.
"""
import math, os, sys, importlib.util

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from hospital_robot_description.msg import FleetTask, RoverTelemetry

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    'fleet_brain', os.path.join(HERE, '..', 'scripts', 'fleet_brain.py'))
fb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fb)

PASS, FAIL = [], []


def check(name, cond, detail=''):
    (PASS if cond else FAIL).append(name)
    print(f'  [{"PASS" if cond else "FAIL"}] {name}' + (f'  — {detail}' if detail else ''))


def straight_path(x0, y0, x1, y1, step=0.25):
    """A path the planner would return for open floor."""
    p = Path()
    p.header.frame_id = 'map'
    n = max(2, int(math.hypot(x1 - x0, y1 - y0) / step))
    for i in range(n + 1):
        t = i / n
        ps = PoseStamped()
        ps.pose.position.x = x0 + t * (x1 - x0)
        ps.pose.position.y = y0 + t * (y1 - y0)
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p


def dogleg(x0, y0, xm, ym, x1, y1):
    """Two straight legs — used to give F2 something to bite on."""
    a = straight_path(x0, y0, xm, ym)
    b = straight_path(xm, ym, x1, y1)
    a.poses.extend(b.poses[1:])
    return a


def tel(rid, x, y, yaw=0.0, wh=480.0, sigma=0.05,
        state=RoverTelemetry.STATE_IDLE):
    t = RoverTelemetry()
    t.rover_id, t.x, t.y, t.yaw = rid, x, y, yaw
    t.battery_wh, t.pose_sigma, t.mission_state = wh, sigma, state
    t.link_state = RoverTelemetry.LINK_OK
    return t


def task(x, y, prio=FleetTask.PRIORITY_NORMAL, kg=2.0):
    m = FleetTask()
    m.goal_x, m.goal_y, m.priority, m.payload_kg = x, y, prio, kg
    return fb.Task(m)


def table(brain, rows):
    print(f'    {"rover":<8}{"F1":>7}{"F2":>7}{"F3":>7}{"F4":>7}'
          f'{"F5":>7}{"F6":>7}{"F7":>7}{"TOTAL":>9}')
    for rid, parts in rows:
        if parts is None:
            print(f'    {rid:<8}{"— VETOED —":>50}')
            continue
        print(f'    {rid:<8}' + ''.join(f'{parts[k]:>7.1f}' for k in
              ('f1', 'f2', 'f3', 'f4', 'f5', 'f6', 'f7')) +
              f'{sum(parts.values()):>9.1f}')


def main():
    rclpy.init()
    brain = fb.FleetBrain()

    # Real stations and real docks, so the numbers below are the ones the
    # running system produces.
    brain.stations = {
        'nurses_station': (1.5, 3.0, 1.57, 1.0),
        'ward_a_bed': (-1.5, 13.0, 1.57, 0.9),
        'ward_b_bed': (9.0, 13.0, 1.57, 0.9),
        'pharmacy': (-12.0, 0.0, 0.0, 0.8),
        'supply_room': (-12.0, 13.5, 1.57, 0.7),
        'surgical_prep': (13.5, -4.6, -1.57, 1.0),
    }
    DOCK = {'rover1': (-13.0, -7.5), 'rover2': (15.5, 2.0),
            'rover3': (-13.0, 7.5), 'rover4': (12.0, 7.5)}
    CAP = {'rover1': 480.0, 'rover2': 455.0, 'rover3': 430.0, 'rover4': 400.0}
    brain.rovers = {}
    for rid, d in DOCK.items():
        r = fb.RoverModel(rid, {'dock': rid, 'dock_pose': [d[0], d[1], 0.0],
                                'battery_wh': CAP[rid]}, 6.0, 1.0)
        r.tel = tel(rid, d[0], d[1])
        r.tel_time = brain._now()
        brain.rovers[rid] = r

    # Mirror the launch configuration: simulation burns energy 12x faster,
    # and the brain predicts on the same scale.
    from rclpy.parameter import Parameter
    brain.set_parameters([Parameter('energy_scale', Parameter.Type.DOUBLE, 12.0)])

    pgm = os.path.join(HERE, '..', 'maps', 'hospital_large.pgm')
    brain.clearance = fb.ClearanceMap(pgm, 0.05, [-15.5, -9.5])

    avail = list(brain.rovers)

    # ── F1: true traversal time orders by path length, not by guesswork ───
    print('\n1. F1  traversal time')
    g = task(1.5, 3.0)
    rows = []
    for rid, r in brain.rovers.items():
        p = straight_path(r.tel.x, r.tel.y, g.x, g.y)
        parts, veto, _ = brain._score(r, g, p, avail)
        rows.append((rid, parts))
    table(brain, rows)
    f1 = {rid: p['f1'] for rid, p in rows}
    lens = {rid: math.hypot(DOCK[rid][0] - 1.5, DOCK[rid][1] - 3.0) for rid in DOCK}
    check('F1 ranks by path length',
          sorted(f1, key=f1.get) == sorted(lens, key=lens.get),
          f'nearest={min(lens, key=lens.get)}')

    # ── F2: same distance, opposite heading ──────────────────────────────
    print('\n2. F2  turn burden (identical path, opposite initial heading)')
    g = task(0.0, -7.5)
    r1, r3 = brain.rovers['rover1'], brain.rovers['rover3']
    r3.tel = tel('rover3', -13.0, -7.5, yaw=0.0)          # facing the goal
    r1.tel = tel('rover1', -13.0, -7.5, yaw=math.pi)      # facing away
    p = straight_path(-13.0, -7.5, 0.0, -7.5)
    a, _, _ = brain._score(r1, g, p, avail)
    b, _, _ = brain._score(r3, g, p, avail)
    table(brain, [('facing away', a), ('facing goal', b)])
    check('F2 penalises the misaligned rover', a['f2'] > b['f2'] + 2.0,
          f'{a["f2"]:.1f} s vs {b["f2"]:.1f} s')
    check('F1 identical for both', abs(a['f1'] - b['f1']) < 1e-6)

    # restore
    r1.tel = tel('rover1', *DOCK['rover1'])
    r3.tel = tel('rover3', *DOCK['rover3'])

    # ── F2 regression: the planner's grid staircase must not read as turning
    print('\n2b. F2  grid staircase (regression)')
    # NavFn returns an 8-connected grid route, so a straight diagonal comes
    # back as a 5 cm staircase alternating +/-45 degrees. Measured raw, that
    # charged a rover ~100 s of "rotation" for driving in a straight line and
    # made F2 a noisy duplicate of F1.
    stair = Path()
    stair.header.frame_id = 'map'
    x = y = 0.0
    for i in range(400):
        for dx, dy in ((0.05, 0.0), (0.05, 0.05)):
            x += dx; y += dy
            ps = PoseStamped()
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            stair.poses.append(ps)
    smooth = straight_path(0.0, 0.0, x, y)
    L_s, turn_s, _ = brain._path_metrics(stair, math.atan2(y, x))
    L_m, turn_m, _ = brain._path_metrics(smooth, math.atan2(y, x))
    print(f'    staircase : length {L_s:6.1f} m   cumulative yaw {turn_s:6.2f} rad')
    print(f'    smooth    : length {L_m:6.1f} m   cumulative yaw {turn_m:6.2f} rad')
    check('staircase yaw stays near zero, not tens of radians', turn_s < 1.0,
          f'{turn_s:.2f} rad over a {L_s:.0f} m straight run')

    # ── F3: contention against a committed path in a corridor ────────────
    print('\n3. F3  contention forecast')
    g = task(-12.0, 0.0)                                   # pharmacy
    cand = straight_path(-13.0, -7.5, -12.0, 0.0)
    clean, _, sched = brain._score(brain.rovers['rover1'], g, cand, avail)
    # now commit rover3 to the same floor at the same time
    brain.rovers['rover3'].schedule = brain._schedule(
        straight_path(-12.0, 0.0, -13.0, -7.5), brain._now())
    busy, _, _ = brain._score(brain.rovers['rover1'], g, cand, avail)
    table(brain, [('no traffic', clean), ('head-on', busy)])
    check('F3 zero when no path is committed', clean['f3'] == 0.0)
    check('F3 fires on a head-on conflict', busy['f3'] > 0.0,
          f'{busy["f3"]:.1f} s predicted delay')
    w = brain.clearance.width_at(-12.5, -4.0)
    check('clearance map reports a real corridor width', 0.5 < w < 12.0,
          f'{w:.2f} m at (-12.5, -4.0)')
    brain.rovers['rover3'].schedule = []

    # ── F4: energy veto on a rover that cannot get home ──────────────────
    print('\n4. F4  energy feasibility')
    g = task(13.5, -4.6, kg=8.0)                           # far corner
    r = brain.rovers['rover4']
    long_path = straight_path(12.0, 7.5, 13.5, -4.6)
    r.tel = tel('rover4', 12.0, 7.5, wh=400.0)
    healthy, veto_h, _ = brain._score(r, g, long_path, avail)
    r.tel = tel('rover4', 12.0, 7.5, wh=85.0)              # nearly flat
    flat, veto_f, _ = brain._score(r, g, long_path, avail)
    table(brain, [('full pack', healthy), ('85 Wh left', flat)])
    check('healthy pack is eligible', healthy is not None)
    check('flat pack is VETOED, not merely penalised', flat is None, veto_f)
    r.tel = tel('rover4', *DOCK['rover4'])

    # ── F5: reliability prior ────────────────────────────────────────────
    print('\n5. F5  reliability prior')
    g = task(1.5, 3.0)
    good = brain.rovers['rover2']
    bad = brain.rovers['rover4']
    for _ in range(4):
        bad.record(False, 0.98)
    p2 = straight_path(good.tel.x, good.tel.y, g.x, g.y)
    p4 = straight_path(bad.tel.x, bad.tel.y, g.x, g.y)
    a, _, _ = brain._score(good, g, p2, avail)
    b, _, _ = brain._score(bad, g, p4, avail)
    table(brain, [('rover2 clean', a), ('rover4 4 fails', b)])
    check('F5 penalises the unreliable rover', b['f5'] > a['f5'] + 10.0,
          f'P(success) {good.p_success:.2f} vs {bad.p_success:.2f}')

    # ── F6: localisation confidence ──────────────────────────────────────
    print('\n6. F6  localisation confidence')
    r = brain.rovers['rover3']
    r.tel = tel('rover3', -13.0, 7.5, sigma=0.05)
    sharp, _, _ = brain._score(r, g, straight_path(-13, 7.5, 1.5, 3.0), avail)
    r.tel = tel('rover3', -13.0, 7.5, sigma=0.95)
    fuzzy, _, _ = brain._score(r, g, straight_path(-13, 7.5, 1.5, 3.0), avail)
    table(brain, [('sigma 0.05', sharp), ('sigma 0.95', fuzzy)])
    check('F6 grows with pose uncertainty', fuzzy['f6'] > sharp['f6'] + 20.0)
    r.tel = tel('rover3', *DOCK['rover3'])

    # ── F7: fleet readiness — the reason to pass over the nearest rover ──
    print('\n7. F7  fleet readiness (opportunity cost)')
    # rover3 is the only rover covering the west/north wing. Committing it
    # should cost more coverage than committing rover1, which has company.
    base = brain._coverage([brain.rovers[i].pos for i in avail])
    d3 = brain._coverage([brain.rovers[i].pos for i in avail if i != 'rover3']) - base
    d1 = brain._coverage([brain.rovers[i].pos for i in avail if i != 'rover1']) - base
    print(f'    baseline coverage {base:.1f} m-weighted')
    print(f'    committing rover3 costs {d3:.1f}   committing rover1 costs {d1:.1f}')
    check('F7 distinguishes load-bearing rovers', abs(d3 - d1) > 0.5,
          f'delta {abs(d3-d1):.1f}')
    check('F7 is never negative', d3 >= 0 and d1 >= 0)

    # ── end-to-end: does the policy ever override nearest-wins? ──────────
    print('\n8. policy overrides nearest-wins when the evidence says so')
    g = task(1.5, 3.0)
    # rover1 is nearest but flat and unreliable; rover2 is further but sound.
    brain.rovers['rover1'].tel = tel('rover1', 3.0, 3.0, wh=150.0, sigma=0.9)
    for _ in range(6):
        brain.rovers['rover1'].record(False, 0.98)
    brain.rovers['rover2'].tel = tel('rover2', 8.0, 3.0, wh=455.0, sigma=0.04)
    rows, best = [], None
    for rid in ('rover1', 'rover2'):
        r = brain.rovers[rid]
        p = straight_path(r.tel.x, r.tel.y, g.x, g.y)
        parts, veto, _ = brain._score(r, g, p, ['rover1', 'rover2'])
        rows.append((rid, parts))
        if parts and (best is None or sum(parts.values()) < best[1]):
            best = (rid, sum(parts.values()))
    table(brain, rows)
    check('closer-but-compromised rover1 does NOT win',
          best is None or best[0] == 'rover2',
          f'winner={best[0] if best else "none eligible"} '
          f'(rover1 is 5 m closer)')

    # ── preemption: only an urgent task may take a rover off a routine one ─
    print('\n9. preemption eligibility')
    # This path does not fire in the demo run, because a rover is always free
    # when the emergency lands. It still has to be right the day one is not.
    for rid in brain.rovers:
        brain.rovers[rid].tel = tel(rid, *DOCK[rid])
        brain.rovers[rid].task = None
        brain.rovers[rid].reloc_until = -fb.BIG
        brain.rovers[rid].reloc_gaveup = False
    busy = brain.rovers['rover1']
    busy.task = task(1.5, 3.0, prio=FleetTask.PRIORITY_ROUTINE)
    now = brain._now()

    routine = task(9.0, 13.0, prio=FleetTask.PRIORITY_NORMAL)
    cands, vetoes = brain._candidates(routine, now)
    check('a routine task cannot preempt a busy rover',
          'rover1' not in [c.rid for c in cands],
          vetoes.get('rover1', ''))

    emergency = task(13.5, -4.6, prio=FleetTask.PRIORITY_EMERGENCY)
    cands, vetoes = brain._candidates(emergency, now)
    check('an EMERGENCY task may preempt a ROUTINE one',
          'rover1' in [c.rid for c in cands],
          f'{len(cands)} candidates')

    busy.task = task(1.5, 3.0, prio=FleetTask.PRIORITY_URGENT)
    cands, vetoes = brain._candidates(emergency, now)
    check('an EMERGENCY task does NOT preempt an URGENT one',
          'rover1' not in [c.rid for c in cands],
          vetoes.get('rover1', ''))
    busy.task = None

    # ── vetoes that must exclude a rover outright ────────────────────────
    print('\n10. hard vetoes')
    r = brain.rovers['rover2']
    r.tel = tel('rover2', *DOCK['rover2'], sigma=2.0)
    cands, vetoes = brain._candidates(routine, now)
    check('a rover that does not know where it is is vetoed',
          'rover2' in vetoes, vetoes.get('rover2', ''))

    r.tel = tel('rover2', *DOCK['rover2'], state=RoverTelemetry.STATE_FAULT)
    cands, vetoes = brain._candidates(routine, now)
    check('a faulted rover is vetoed', 'rover2' in vetoes,
          vetoes.get('rover2', ''))

    r.tel = tel('rover2', *DOCK['rover2'])
    r.tel_time = now - 99.0
    cands, vetoes = brain._candidates(routine, now)
    check('a rover with stale telemetry is vetoed', 'rover2' in vetoes,
          vetoes.get('rover2', ''))

    print(f'\n{"="*66}\n  {len(PASS)} passed, {len(FAIL)} failed')
    if FAIL:
        print('  failures: ' + ', '.join(FAIL))
    brain.destroy_node()
    rclpy.shutdown()
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
