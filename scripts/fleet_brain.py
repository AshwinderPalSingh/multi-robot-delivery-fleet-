#!/usr/bin/env python3
"""
fleet_brain — the single decision-maker for the whole rover fleet.

ARCHITECTURE
────────────
There is exactly one of these. It runs on the static compute node (Raspberry
Pi 5 or Jetson Orin Nano) together with every rover's Nav2 stack, the shared
map server, and one rover_link per rover. The rovers themselves hold no
planner, no costmap, no map and no goal — an ESP32 that receives a velocity
pair and turns wheels. Nothing about the fleet's behaviour lives on a rover,
so the fleet can be retuned, re-tasked or debugged entirely from one machine.

That is the whole reason this is worth doing centrally: three of the seven
factors below are simply not computable by a rover bidding on its own
behalf, because they depend on what the OTHER rovers are doing and on fleet
history no individual rover has.

THE ASSIGNMENT POLICY
─────────────────────
Nearest-rover-wins is the obvious policy and it is wrong often enough to
matter. This brain scores every eligible rover on seven factors, all reduced
to EFFECTIVE SECONDS so the weighted sum is a physically meaningful estimate
of what the mission will cost the fleet:

  F1  Traversal time      True Nav2 path length / cruise speed. Not Euclidean
                          distance — a rover four metres away on the far side
                          of a wall is not near.

  F2  Turn burden         Differential-drive robots pay for rotation in time,
                          not distance. Initial heading error plus cumulative
                          yaw along the path, divided by turn rate. A rover
                          one metre closer but facing the wrong way down a
                          corridor is genuinely further away.

  F3  Contention forecast Because the brain holds every committed path, it
                          can predict where two rovers will want the same
                          floor at the same time — before dispatching. The
                          candidate path is bucketed in space and time
                          against every committed path; overlaps are weighted
                          by how narrow the corridor is there. Below
                          pinch_width_m two rovers physically cannot pass, so
                          an overlap is not a slowdown, it is a deadlock.
                          CENTRALISED-ONLY: a bidding rover cannot know this.

  F4  Energy feasibility  Not battery percentage. The predicted watt-hours for
                          the mission PLUS the return leg to the nearest dock,
                          against the usable pack above the reserve. A rover
                          that cannot get back is VETOED, not merely penalised
                          — stranding a rover mid-corridor blocks the fleet.

  F5  Reliability prior   A Beta(alpha, beta) posterior per rover on "does it
                          finish what it is given", updated after every
                          mission and decayed so old history stops dominating
                          a repaired rover. A rover whose motor is degrading
                          is quietly de-prioritised for hard jobs before a
                          human notices. CENTRALISED-ONLY: needs fleet memory.

  F6  Localisation conf.  The trace of the rover's AMCL covariance. A rover
                          that does not know where it is will abort a long
                          delivery. Cheap lidar in a bare corridor genuinely
                          loses track, so this is a real hardware failure
                          mode and not a simulation artefact.

  F7  Fleet readiness     The opportunity cost of committing this rover.
                          Stations carry a demand weight — the prior on where
                          the NEXT call comes from. Sending the nearest rover
                          sometimes strips cover from a whole wing, so the
                          brain measures how much worse the fleet answers an
                          unknown future request and will deliberately pass
                          over the nearest rover to stay spread out.
                          CENTRALISED-ONLY: no agent can evaluate fleet-wide
                          coverage from inside.

Every decision is published in full on /fleet/assignment — per-candidate,
per-factor, including vetoes — so any choice can be replayed and defended
afterwards, which matters most exactly when the brain does NOT pick the
closest rover.
"""

import math
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import (PoseStamped, Point, Twist,
                               PoseWithCovarianceStamped)
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray

from hospital_robot_description.msg import (AssignmentDecision, FleetTask,
                                            RoverTelemetry)

BIG = 1.0e6


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


# ═══════════════════════════════════════════════════════════════════════════
class ClearanceMap:
    """Corridor half-width lookup, built once from the static map.

    Factor F3 needs to know how wide the floor is at a point: two rovers
    meeting in a 4 m atrium is a non-event, two meeting in a 1.6 m corridor is
    a deadlock. A coarse grid is deliberate — 0.25 m is far finer than the
    decision needs, and it keeps the whole structure a few thousand cells so
    it can be rebuilt on a Pi at boot in well under a second.
    """

    def __init__(self, pgm_path, resolution, origin, coarse=5, logger=None):
        raw = open(pgm_path, 'rb').read()
        i = 0
        for _ in range(4):                     # magic, comment, dims, maxval
            i = raw.index(b'\n', i) + 1
        w, h = [int(t) for t in raw[:i].decode().split('\n')[2].split()]
        g = np.flipud(np.frombuffer(raw[i:], np.uint8).reshape(h, w))

        blocked = (g != 254)
        ch, cw = h // coarse, w // coarse
        blk = blocked[:ch * coarse, :cw * coarse]
        # A coarse cell counts as blocked if ANY fine cell in it is — the
        # conservative direction, which is the right one for a safety margin.
        blk = blk.reshape(ch, coarse, cw, coarse).any(axis=(1, 3))

        self.res = resolution * coarse
        self.ox, self.oy = origin[0], origin[1]
        self.h, self.w = ch, cw

        # Half-width by successive erosion: the largest disc that fits.
        levels = [0.4, 0.7, 1.0, 1.3, 1.7, 2.2, 3.0]
        clear = np.zeros((ch, cw), np.float32)
        free = ~blk
        for lv in levels:
            r = lv / self.res
            R = int(np.ceil(r))
            ok = free.copy()
            for dv in range(-R, R + 1):
                for du in range(-R, R + 1):
                    if dv * dv + du * du > r * r:
                        continue
                    sh = np.zeros_like(free)
                    v0, v1 = max(0, dv), ch + min(0, dv)
                    u0, u1 = max(0, du), cw + min(0, du)
                    sh[v0:v1, u0:u1] = free[v0 - dv:v1 - dv, u0 - du:u1 - du]
                    ok &= sh
            clear[ok] = lv
        self.clear = clear
        if logger:
            logger.info(f'clearance map {cw}x{ch} @ {self.res:.2f} m '
                        f'(max half-width {clear.max():.1f} m)')

    def width_at(self, x, y):
        """Free-floor width in metres at a world point (2x the half-width)."""
        u = int((x - self.ox) / self.res)
        v = int((y - self.oy) / self.res)
        if not (0 <= u < self.w and 0 <= v < self.h):
            return 0.0
        return float(self.clear[v, u]) * 2.0


# ═══════════════════════════════════════════════════════════════════════════
class RoverModel:
    """The brain's complete picture of one rover. Rovers store none of this."""

    def __init__(self, rid, cfg, alpha0, beta0):
        self.rid = rid
        self.dock_name = cfg.get('dock', '')
        self.dock_pose = cfg.get('dock_pose', [0.0, 0.0, 0.0])
        self.capacity_wh = float(cfg.get('battery_wh', 480.0))

        self.tel = None
        self.tel_time = -BIG
        self.task = None                 # Task currently executing
        self.goal_handle = None
        self.path = None                 # committed nav_msgs/Path
        self.path_t0 = 0.0
        self.schedule = []               # [(x, y, absolute time)]

        # Beta posterior on mission completion. Seeded optimistic but not
        # certain, so a couple of early failures actually move it.
        self.sigma_bad_since = None      # when pose uncertainty first exceeded the veto
        self.reloc_until = -BIG          # rotating for global relocalisation until this time
        self.reloc_attempts = 0
        self.reloc_gaveup = False

        self.alpha = alpha0
        self.beta = beta0
        self.missions = 0
        self.failures = 0

    @property
    def online(self):
        return self.tel is not None

    @property
    def pos(self):
        return (self.tel.x, self.tel.y) if self.tel else (0.0, 0.0)

    @property
    def p_success(self):
        return self.alpha / max(1e-6, self.alpha + self.beta)

    def record(self, ok, decay):
        self.alpha *= decay
        self.beta *= decay
        self.missions += 1
        if ok:
            self.alpha += 1.0
        else:
            self.beta += 1.0
            self.failures += 1


class Task:
    def __init__(self, msg: FleetTask):
        self.id = msg.task_id or f't{uuid.uuid4().hex[:6]}'
        self.station = msg.station
        self.x, self.y, self.yaw = msg.goal_x, msg.goal_y, msg.goal_yaw
        self.priority = msg.priority
        self.payload_kg = msg.payload_kg
        self.attempts = 0
        self.last_fail = -BIG
        self.assigned_to = None
        # Delivery tasks are open to any rover. Maintenance errands -- docking
        # to charge, or a sortie to re-seed a rover's own AMCL -- are bound to
        # one rover and are meaningless performed by another, so they carry an
        # owner and never enter the shared queue.
        self.owner = None


class BidRound:
    """One asynchronous round of planner queries.

    Every candidate's path is requested at once and scored only when all have
    answered or the round times out. Nothing blocks: the decision timer keeps
    running, so telemetry and task submission stay live while the planners
    think.
    """

    def __init__(self, task, candidates, deadline):
        self.task = task
        self.candidates = candidates
        self.deadline = deadline
        self.paths = {}
        self.failed = set()

    @property
    def complete(self):
        return len(self.paths) + len(self.failed) >= len(self.candidates)


# ═══════════════════════════════════════════════════════════════════════════
class FleetBrain(Node):

    def __init__(self):
        super().__init__('fleet_brain')
        self.cb = ReentrantCallbackGroup()

        # use_sim_time is declared by rclpy itself; redeclaring it raises.
        self.declare_parameters('', [
            ('rovers', ['rover1', 'rover2', 'rover3', 'rover4']),
            ('cruise_speed', 0.35), ('turn_rate', 1.0),
            ('w1_path', 1.0), ('w2_turn', 1.0), ('w3_contention', 1.5),
            ('w4_energy', 60.0), ('w5_reliability', 45.0),
            ('w6_localization', 30.0), ('w7_coverage', 0.25),
            ('veto_link_stale_s', 3.0), ('veto_pose_sigma', 1.2),
            # A rover vetoed on pose uncertainty is stuck: AMCL only sharpens
            # with motion, and a vetoed rover is never given any. After this
            # long it is sent on a relocalisation sortie to its dock.
            ('reloc_after_s', 20.0),
            ('reloc_spin_s', 22.0),        # long enough for the arc to cover a few metres
            ('reloc_spin_rate', 0.6),      # rad/s
            ('reloc_creep_v', 0.12),       # m/s of translation during recovery
            ('reloc_min_clear_m', 1.30),   # forward room required before creeping
            # Above this the rover is genuinely lost and its estimate is worth
            # nothing, so scattering the particles costs nothing. BELOW it the
            # estimate is merely loose and scattering actively destroys it --
            # measured: rover2 at sigma 1.26 was scattered and came back at
            # 2.43. Mild cases get motion only.
            ('lost_pose_sigma', 3.0),
            ('max_reloc_attempts', 3),
            ('energy_reserve_frac', 0.20), ('max_task_attempts', 2),
            ('reassign_cooldown_s', 25.0),
            ('energy_wh_per_m', 0.12), ('energy_wh_per_rad', 0.05),
            # 1.0 on hardware. In simulation rover_link drains the pack
            # faster than real physics so the energy factor is exercised in
            # minutes rather than hours; the brain must predict on the same
            # scale or it will forecast missions it cannot actually afford.
            ('energy_scale', 1.0),
            ('energy_idle_w', 15.0), ('energy_wh_per_kg_m', 0.0006),
            ('reliability_alpha0', 6.0), ('reliability_beta0', 1.0),
            ('reliability_decay', 0.98),
            ('contention_horizon_s', 60.0), ('contention_cell_m', 0.5),
            ('contention_time_bucket_s', 5.0), ('pinch_width_m', 1.9),
            # Arc length the path is resampled to before headings are taken.
            # NavFn returns an 8-connected grid path, so a straight diagonal
            # arrives as a staircase alternating +/-45 deg every 5 cm. Summing
            # raw segment-to-segment yaw over that measures grid quantisation,
            # not rotation the rover will ever perform.
            ('turn_sample_m', 0.75),
            # Headings are taken across this many resampled points rather
            # than adjacent ones. Measured on a synthetic 8-connected
            # staircase, resample-0.75 m + window-2 reports 0.02 rad of yaw
            # for a straight 48 m diagonal (adjacent-point differencing
            # reports 3.7) while still scoring a genuine right-angle turn at
            # exactly 1.57. See test/test_policy.py case 2b.
            ('turn_window', 2),
            ('decision_period_s', 1.0), ('telemetry_timeout_s', 5.0),
            ('bid_timeout_s', 4.0),
            ('map_pgm', ''), ('map_resolution', 0.05),
            ('map_origin', [-15.5, -9.5]),
            ('station_names', ['']), ('station_data', [0.0]),
            ('rover_docks', ['']), ('rover_dock_poses', [0.0]),
            ('rover_batteries', [0.0]),
            ('auto_dock_frac', 0.28),
        ])
        g = lambda n: self.get_parameter(n).value
        self.G = g

        self.cruise = float(g('cruise_speed'))
        self.turn_rate = float(g('turn_rate'))
        self.reserve = float(g('energy_reserve_frac'))

        # ── stations: name -> (x, y, yaw, demand) ─────────────────────────
        names = list(g('station_names'))
        flat = list(g('station_data'))
        self.stations = {n: tuple(flat[i * 4:i * 4 + 4])
                         for i, n in enumerate(names) if n}

        # ── rovers ────────────────────────────────────────────────────────
        rids = list(g('rovers'))
        docks = list(g('rover_docks'))
        dposes = list(g('rover_dock_poses'))
        batts = list(g('rover_batteries'))
        a0, b0 = float(g('reliability_alpha0')), float(g('reliability_beta0'))
        self.rovers = {}
        for i, rid in enumerate(rids):
            self.rovers[rid] = RoverModel(rid, {
                'dock': docks[i] if i < len(docks) else '',
                'dock_pose': dposes[i * 3:i * 3 + 3] if len(dposes) > i * 3 else [0, 0, 0],
                'battery_wh': batts[i] if i < len(batts) else 480.0,
            }, a0, b0)

        # ── clearance map for the contention forecast ─────────────────────
        self.clearance = None
        pgm = g('map_pgm')
        if pgm:
            try:
                self.clearance = ClearanceMap(pgm, float(g('map_resolution')),
                                              list(g('map_origin')),
                                              logger=self.get_logger())
            except Exception as e:
                self.get_logger().warn(f'clearance map unavailable ({e}); '
                                       f'F3 will fall back to a flat penalty')

        # ── ROS interfaces ────────────────────────────────────────────────
        self.create_subscription(FleetTask, '/fleet/task', self._on_task, 10)
        for rid in self.rovers:
            self.create_subscription(
                RoverTelemetry, f'/{rid}/telemetry',
                lambda m, r=rid: self._on_telemetry(r, m), 10)

        self.pub_decision = self.create_publisher(AssignmentDecision,
                                                  '/fleet/assignment', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/fleet/markers', 10)
        # Re-seeding AMCL is the brain's job, not the rover's. A docked rover
        # is mechanically aligned to a surveyed pose, so the dock is the one
        # place in the ward where the fleet knows exactly where a rover is.
        # Recovering a genuinely lost rover means scattering AMCL's particles
        # over the whole map and giving the filter motion to resample on. The
        # brain drives that rotation directly through rover_link -- the same
        # path Nav2's velocities take -- because Nav2 itself cannot be trusted
        # to move a rover whose pose is wrong.
        self.cli_globalloc = {
            rid: self.create_client(Empty,
                                    f'/{rid}/reinitialize_global_localization',
                                    callback_group=self.cb)
            for rid in self.rovers}
        self.pub_cmdvel = {
            rid: self.create_publisher(Twist, f'/{rid}/override_cmd', 10)
            for rid in self.rovers}
        # The scans live on the brain anyway, which is what makes a safe
        # reactive creep possible without a costmap: recovery motion is
        # gated directly on the forward sector of the rover's own lidar.
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST)
        self.fwd_clear = {rid: 0.0 for rid in self.rovers}
        for rid in self.rovers:
            self.create_subscription(
                LaserScan, f'/{rid}/scan',
                lambda m, r=rid: self._on_scan(r, m), scan_qos,
                callback_group=self.cb)
        self.pub_initpose = {
            rid: self.create_publisher(PoseWithCovarianceStamped,
                                       f'/{rid}/initialpose', 10)
            for rid in self.rovers}

        self.plan_clients = {
            rid: ActionClient(self, ComputePathToPose,
                              f'/{rid}/compute_path_to_pose', callback_group=self.cb)
            for rid in self.rovers}
        self.nav_clients = {
            rid: ActionClient(self, NavigateToPose,
                              f'/{rid}/navigate_to_pose', callback_group=self.cb)
            for rid in self.rovers}

        self.queue = []
        self.round = None
        self.completed = 0
        self.reassigned = 0

        self.create_timer(float(g('decision_period_s')), self._tick,
                          callback_group=self.cb)
        self.create_timer(1.0, self._publish_markers, callback_group=self.cb)
        # the override must be refreshed faster than rover_link's 0.4 s
        # override timeout, or the rotation drops back to Nav2 mid-recovery
        self.create_timer(0.1, self._spin_tick, callback_group=self.cb)

        self.get_logger().info(
            '╔══════════════════════════════════════════════════════════╗\n'
            '║  GFORCE CENTRAL BRAIN — centralised fleet controller      ║\n'
            '╠══════════════════════════════════════════════════════════╣\n'
            f'║  rovers    : {", ".join(rids):<43} ║\n'
            f'║  stations  : {len(self.stations):<43} ║\n'
            '║  policy    : 7-factor weighted assignment                 ║\n'
            '║  rovers run: no planner, no costmap, no autonomy          ║\n'
            '╚══════════════════════════════════════════════════════════╝')

    # ── clock ─────────────────────────────────────────────────────────────
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ── inputs ────────────────────────────────────────────────────────────
    def _on_telemetry(self, rid, msg):
        r = self.rovers[rid]
        r.tel = msg
        r.tel_time = self._now()

    def _on_scan(self, rid, msg: LaserScan):
        """Cache the closest return in the forward +/-25 degree sector."""
        n = len(msg.ranges)
        if not n or msg.angle_increment == 0.0:
            return
        half = math.radians(25.0)
        i0 = int((-half - msg.angle_min) / msg.angle_increment)
        i1 = int((half - msg.angle_min) / msg.angle_increment)
        i0, i1 = max(0, min(i0, i1)), min(n, max(i0, i1) + 1)
        best = float('inf')
        for r in msg.ranges[i0:i1]:
            if msg.range_min < r < msg.range_max and r < best:
                best = r
        self.fwd_clear[rid] = best if best < float('inf') else msg.range_max

    def _on_task(self, msg):
        t = Task(msg)
        if t.station and t.station in self.stations:
            sx, sy, syaw, _ = self.stations[t.station]
            t.x, t.y, t.yaw = sx, sy, syaw
        self.queue.append(t)
        label = t.station or f'({t.x:.1f}, {t.y:.1f})'
        self.get_logger().info(
            f'TASK [{t.id}] -> {label}  priority={t.priority}  '
            f'queue={len(self.queue)}')

    # ═════════════════════════════════════════════════════════════════════
    #  Main loop
    # ═════════════════════════════════════════════════════════════════════
    def _tick(self):
        now = self._now()

        # 1. drop rovers whose telemetry has gone silent
        for r in self.rovers.values():
            if r.online and (now - r.tel_time) > float(self.G('telemetry_timeout_s')):
                if r.task:
                    self.get_logger().warn(
                        f'{r.rid}: telemetry lost mid-mission; requeueing '
                        f'[{r.task.id}]')
                    self._release(r, ok=False, requeue=True)
                r.tel = None

        # 2. finish an in-flight bid round
        if self.round is not None:
            if self.round.complete or now > self.round.deadline:
                rnd, self.round = self.round, None
                self._resolve(rnd)
            return

        # 3. send flat rovers home before they strand themselves, and rescue
        #    any rover that has been vetoed on localisation long enough to be
        #    permanently stuck
        self._auto_dock()
        self._relocalize(now)

        # 4. start a round for the highest-priority ready task
        task = self._next_task(now)
        if task is not None:
            self._open_round(task, now)

    def _next_task(self, now):
        ready = [t for t in self.queue
                 if t.assigned_to is None
                 and t.attempts < int(self.G('max_task_attempts'))
                 and (now - t.last_fail) > float(self.G('reassign_cooldown_s'))]
        if not ready:
            return None
        ready.sort(key=lambda t: (-t.priority, t.attempts))
        return ready[0]

    def _auto_dock(self):
        frac = float(self.G('auto_dock_frac'))
        for r in self.rovers.values():
            if r.task is not None or not r.online:
                continue
            if r.tel.battery_wh > frac * r.capacity_wh:
                continue
            dx, dy, dyaw = r.dock_pose
            if math.hypot(r.pos[0] - dx, r.pos[1] - dy) < 1.0:
                continue
            self.get_logger().warn(
                f'{r.rid}: pack at {r.tel.battery_wh:.0f} Wh '
                f'({100*r.tel.battery_wh/r.capacity_wh:.0f}%), returning to '
                f'{r.dock_name}')
            t = Task(FleetTask())
            t.id = f'dock-{r.rid}'
            t.x, t.y, t.yaw = dx, dy, dyaw
            t.station = r.dock_name
            t.owner = r.rid
            self._dispatch(r, t, recharge=True)

    def _relocalize(self, now):
        """Recover a rover the pose-sigma veto has locked out.

        The first version of this drove the rover to its dock and re-seeded
        AMCL there. That works only while the rover is merely UNCERTAIN. It
        cannot work once the rover is actually lost, and measurement showed
        exactly that: rover3 reporting sigma 2.99 was 3.35 m from where it
        believed it was -- convinced it stood inside the pharmacy while it was
        outside in the corridor. Nav2 planned the drive home from that false
        pose and the controller aborted every time, because the route bore no
        relation to the floor the rover was on.

        So the recovery is the kidnapped-robot one instead: scatter AMCL's
        particles across the entire map, then rotate in place so the filter
        gets the motion it needs to resample and collapse onto the true pose.
        Rotation is deliberate -- it cannot collide with anything, which
        matters when by definition nobody knows where the rover is.
        """
        lim = float(self.G('veto_pose_sigma'))
        after = float(self.G('reloc_after_s'))
        max_try = int(self.G('max_reloc_attempts'))

        for r in self.rovers.values():
            if not r.online or r.task is not None or now < r.reloc_until:
                continue

            if r.tel.pose_sigma <= lim:
                if r.reloc_attempts:
                    self.get_logger().info(
                        f'{r.rid}: relocalised — sigma back to '
                        f'{r.tel.pose_sigma:.2f} m after {r.reloc_attempts} '
                        f'attempt(s)')
                r.sigma_bad_since = None
                r.reloc_attempts = 0
                r.reloc_gaveup = False
                continue

            if r.reloc_gaveup:
                continue
            if r.sigma_bad_since is None:
                r.sigma_bad_since = now
                continue
            if now - r.sigma_bad_since < after:
                continue

            if r.reloc_attempts >= max_try:
                r.reloc_gaveup = True
                lost = float(self.G('lost_pose_sigma'))
                why = ('lost' if r.tel.pose_sigma >= lost else
                       'stuck loose — probably somewhere featureless')
                self.get_logger().error(
                    f'{r.rid}: still {why} (sigma {r.tel.pose_sigma:.2f} m) '
                    f'after {max_try} recovery attempts — needs a human. '
                    f'Correct it with the 2D Pose Estimate tool for {r.rid} '
                    f'in RViz.')
                continue

            r.reloc_attempts += 1
            r.sigma_bad_since = None
            r.reloc_until = now + float(self.G('reloc_spin_s'))

            # Two tiers, because the cure is worse than the disease for a rover
            # that is merely loose rather than lost. A loose filter tightens on
            # its own once given motion; scattering it throws away a usable
            # estimate and starts from nothing.
            #
            # Scattering is gated on sigma ALONE, never on attempt count. An
            # earlier version also escalated to a scatter on the second try,
            # which promptly scattered rover3 at sigma 1.41 -- a rover that was
            # 0.6 m out and recovering fine. If repeated arcs cannot tighten a
            # loose filter, the rover is somewhere featureless and scattering
            # will not help either; that case escalates to a human instead.
            lost = float(self.G('lost_pose_sigma'))
            scatter = r.tel.pose_sigma >= lost

            if scatter:
                cli = self.cli_globalloc[r.rid]
                if cli.service_is_ready():
                    cli.call_async(Empty.Request())
                    self.get_logger().warn(
                        f'{r.rid}: sigma {r.tel.pose_sigma:.2f} m, past the '
                        f'{lost:.1f} m lost threshold — scattering AMCL '
                        f'particles and driving a recovery arc '
                        f'(attempt {r.reloc_attempts}/{max_try})')
                else:
                    self.get_logger().warn(
                        f'{r.rid}: global relocalisation service not up; '
                        f'driving the arc anyway')
            else:
                self.get_logger().warn(
                    f'{r.rid}: sigma {r.tel.pose_sigma:.2f} m over the '
                    f'{lim:.2f} m limit — driving a recovery arc to give the '
                    f'filter motion (attempt {r.reloc_attempts}/{max_try}, '
                    f'particles kept)')

    def _spin_tick(self):
        """Drive the recovery motion, and stop it cleanly.

        Rotation alone does NOT recover a lost rover, which measurement made
        plain: after scattering the particles, spinning in place left sigma at
        8.4 m. The reason is that turning on the spot never changes the
        viewpoint. Every particle sees the same scan it saw before, so any
        wrong hypothesis that already matched still matches, and the filter has
        nothing to eliminate.

        Translation is what kills wrong hypotheses. So the recovery drives a
        slow ARC -- turning and creeping at once -- which both changes the
        viewpoint and sweeps the heading. It is gated on the forward sector of
        the rover's own lidar: with no costmap to trust and no idea where the
        rover is, the live scan is the only safe thing to steer by. If the way
        ahead is not clear the rover turns on the spot until it is.
        """
        now = self._now()
        rate = float(self.G('reloc_spin_rate'))
        creep = float(self.G('reloc_creep_v'))
        need = float(self.G('reloc_min_clear_m'))
        for r in self.rovers.values():
            if r.reloc_until <= -BIG / 2:
                continue
            t = Twist()
            if now < r.reloc_until:
                if self.fwd_clear.get(r.rid, 0.0) >= need:
                    t.linear.x = creep
                    t.angular.z = rate * 0.5
                else:
                    t.angular.z = rate          # blocked ahead: turn only
                self.pub_cmdvel[r.rid].publish(t)
            elif now < r.reloc_until + 1.0:
                self.pub_cmdvel[r.rid].publish(t)      # zero, then stop publishing
            else:
                r.reloc_until = -BIG

    # ═════════════════════════════════════════════════════════════════════
    #  Bidding
    # ═════════════════════════════════════════════════════════════════════
    def _candidates(self, task, now):
        """Eligible rovers, with the reason any rover was excluded."""
        out, vetoes = [], {}
        for r in self.rovers.values():
            if not r.online:
                vetoes[r.rid] = 'offline'
            elif (now - r.tel_time) > float(self.G('veto_link_stale_s')):
                vetoes[r.rid] = 'link stale'
            elif r.tel.mission_state == RoverTelemetry.STATE_FAULT:
                vetoes[r.rid] = 'fault'
            elif r.reloc_until > now:
                vetoes[r.rid] = 'relocalising'
            elif r.reloc_gaveup:
                vetoes[r.rid] = 'lost — needs manual pose estimate'
            elif r.tel.pose_sigma > float(self.G('veto_pose_sigma')):
                vetoes[r.rid] = f'lost (sigma {r.tel.pose_sigma:.2f} m)'
            elif r.task is not None:
                # Busy — unless this task outranks what it is doing by a clear
                # margin. Preemption is a centralised power: no rover can
                # decide on its own that its errand matters less than another.
                if task.priority >= FleetTask.PRIORITY_URGENT and \
                        r.task.priority <= task.priority - 2:
                    out.append(r)
                else:
                    vetoes[r.rid] = f'busy [{r.task.id}]'
            else:
                out.append(r)
        return out, vetoes

    def _open_round(self, task, now):
        cands, vetoes = self._candidates(task, now)
        if not cands:
            return
        self.round = BidRound(task, cands, now + float(self.G('bid_timeout_s')))
        self.round.vetoes = vetoes

        goal_pose = self._pose(task.x, task.y, task.yaw)
        for r in cands:
            client = self.plan_clients[r.rid]
            if not client.server_is_ready():
                self.round.failed.add(r.rid)
                continue
            g = ComputePathToPose.Goal()
            g.goal = goal_pose
            g.start = self._pose(r.tel.x, r.tel.y, r.tel.yaw)
            g.use_start = True
            g.planner_id = 'GridBased'
            fut = client.send_goal_async(g)
            fut.add_done_callback(
                lambda f, rid=r.rid, rnd=self.round: self._on_plan_accepted(f, rid, rnd))

    def _on_plan_accepted(self, fut, rid, rnd):
        try:
            handle = fut.result()
        except Exception:
            rnd.failed.add(rid)
            return
        if not handle.accepted:
            rnd.failed.add(rid)
            return
        handle.get_result_async().add_done_callback(
            lambda f, r=rid, n=rnd: self._on_plan_result(f, r, n))

    def _on_plan_result(self, fut, rid, rnd):
        try:
            res = fut.result()
        except Exception:
            rnd.failed.add(rid)
            return
        path = getattr(res.result, 'path', None)
        if res.status != GoalStatus.STATUS_SUCCEEDED or path is None or len(path.poses) < 2:
            rnd.failed.add(rid)
        else:
            rnd.paths[rid] = path

    # ═════════════════════════════════════════════════════════════════════
    #  Scoring
    # ═════════════════════════════════════════════════════════════════════
    @staticmethod
    def _resample(pts, step):
        """Thin a polyline to roughly fixed arc-length spacing."""
        if len(pts) < 3:
            return list(pts)
        out = [pts[0]]
        acc = 0.0
        for a, b in zip(pts, pts[1:]):
            acc += math.hypot(b[0] - a[0], b[1] - a[1])
            if acc >= step:
                out.append(b)
                acc = 0.0
        if out[-1] != pts[-1]:
            out.append(pts[-1])
        return out

    def _path_metrics(self, path, start_yaw):
        """Length, real cumulative yaw, and initial heading error.

        Length is taken on the raw path, where every segment is real distance.
        Yaw is taken on a RESAMPLED path: NavFn hands back an 8-connected grid
        route, so a straight diagonal corridor arrives as a 5 cm staircase that
        turns +/-45 degrees at every step. Summing that raw would charge a
        rover a hundred radians for driving in a straight line -- which is
        exactly what it did before this was resampled, making F2 a noisy second
        copy of F1 rather than an independent factor.
        """
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        length = 0.0
        for a, b in zip(pts, pts[1:]):
            length += math.hypot(b[0] - a[0], b[1] - a[1])

        coarse = self._resample(pts, float(self.G('turn_sample_m')))
        win = max(1, int(self.G('turn_window')))
        turn = 0.0
        prev_h = first_h = None
        for i in range(len(coarse) - win):
            a, b = coarse[i], coarse[i + win]
            dx, dy = b[0] - a[0], b[1] - a[1]
            if math.hypot(dx, dy) < 1e-4:
                continue
            h = math.atan2(dy, dx)
            if first_h is None:
                first_h = h
            if prev_h is not None:
                turn += abs(math.atan2(math.sin(h - prev_h), math.cos(h - prev_h)))
            prev_h = h
        if first_h is None:
            return length, 0.0, 0.0
        e = math.atan2(math.sin(first_h - start_yaw), math.cos(first_h - start_yaw))
        return length, turn, abs(e)

    def _schedule(self, path, t0):
        """Sample a path into (x, y, absolute-time) at the contention cell size."""
        cell = float(self.G('contention_cell_m'))
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        out, acc, last = [], 0.0, None
        for pt in pts:
            if last is not None:
                acc += math.hypot(pt[0] - last[0], pt[1] - last[1])
            last = pt
            if not out or acc - out[-1][2] >= cell:
                out.append((pt[0], pt[1], acc))
        return [(x, y, t0 + s / self.cruise) for x, y, s in out]

    def _contention(self, cand_sched, exclude_rid):
        """Predicted seconds lost to meeting another rover on committed floor.

        Both schedules are bucketed by (space, time). A shared bucket means
        the two rovers want the same floor within the same few seconds. What
        it costs depends entirely on how wide that floor is, which is why the
        clearance map exists: in a 4 m atrium two rovers pass without
        noticing, below pinch_width_m they cannot pass at all.
        """
        cell = float(self.G('contention_cell_m'))
        tb = float(self.G('contention_time_bucket_s'))
        pinch = float(self.G('pinch_width_m'))
        horizon = float(self.G('contention_horizon_s'))
        now = self._now()

        committed = {}
        for r in self.rovers.values():
            if r.rid == exclude_rid or not r.schedule:
                continue
            for x, y, t in r.schedule:
                if t < now or t > now + horizon:
                    continue
                committed.setdefault((int(x / cell), int(y / cell), int(t / tb)),
                                     []).append((x, y))

        if not committed:
            return 0.0

        worst = {}
        for x, y, t in cand_sched:
            if t > now + horizon:
                break
            kx, ky, kt = int(x / cell), int(y / cell), int(t / tb)
            for dt in (-1, 0, 1):
                if (kx, ky, kt + dt) not in committed:
                    continue
                width = self.clearance.width_at(x, y) if self.clearance else pinch
                if width < pinch:
                    sev = 1.0                       # cannot pass — deadlock risk
                elif width < 3.0:
                    sev = 0.5 * (3.0 - width) / max(0.1, 3.0 - pinch)
                else:
                    sev = 0.05                      # open floor, minor slowdown
                worst[kt] = max(worst.get(kt, 0.0), sev)
        return sum(worst.values()) * tb

    def _energy(self, r, task, length, turn):
        """Watt-hours for the mission plus the return leg, and the margin."""
        wm = float(self.G('energy_wh_per_m'))
        wr = float(self.G('energy_wh_per_rad'))
        idle = float(self.G('energy_idle_w'))
        wkg = float(self.G('energy_wh_per_kg_m'))

        scale = float(self.G('energy_scale'))
        t_mission = length / self.cruise + turn / self.turn_rate
        e_mission = (wm * length + wr * turn + wkg * task.payload_kg * length) * scale
        e_mission += idle * t_mission / 3600.0        # idle draw is not scaled
        # Return leg: straight-line to this rover's dock, inflated for the
        # detour a real route takes. Cheap, and it only has to be right enough
        # to keep a rover from stranding itself.
        dx, dy, _ = r.dock_pose
        e_return = wm * math.hypot(task.x - dx, task.y - dy) * 1.35 * scale

        usable = max(1e-3, r.tel.battery_wh - self.reserve * r.capacity_wh)
        need = e_mission + e_return
        return need, usable, 1.0 - need / usable

    def _coverage(self, positions):
        """Demand-weighted distance from the nearest available rover to each
        station. Lower is a better-covered ward."""
        if not positions:
            return BIG
        tot = 0.0
        for x, y, _yaw, dem in self.stations.values():
            tot += dem * min(math.hypot(px - x, py - y) for px, py in positions)
        return tot

    def _score(self, r, task, path, avail_ids):
        length, turn, head_err = self._path_metrics(path, r.tel.yaw)

        f1 = length / self.cruise
        f2 = (head_err + turn) / self.turn_rate

        sched = self._schedule(path, self._now())
        f3 = self._contention(sched, r.rid)

        need, usable, margin = self._energy(r, task, length, turn)
        if margin <= 0.0:
            return None, (f'energy short by {need - usable:.0f} Wh'), sched
        f4 = float(self.G('w4_energy')) * (1.0 - margin) ** 2

        f5 = float(self.G('w5_reliability')) * (-math.log(max(1e-3, r.p_success)))
        f6 = float(self.G('w6_localization')) * r.tel.pose_sigma

        # F7: how much worse the ward is covered while this rover is busy.
        base_pos = [self.rovers[i].pos for i in avail_ids]
        rest_pos = [self.rovers[i].pos for i in avail_ids if i != r.rid]
        d_cov = min(200.0, max(0.0, self._coverage(rest_pos) - self._coverage(base_pos)))
        f7 = float(self.G('w7_coverage')) * d_cov / self.cruise

        parts = {
            'f1': f1 * float(self.G('w1_path')),
            'f2': f2 * float(self.G('w2_turn')),
            'f3': f3 * float(self.G('w3_contention')),
            'f4': f4, 'f5': f5, 'f6': f6, 'f7': f7,
        }
        return parts, '', sched

    # ═════════════════════════════════════════════════════════════════════
    #  Decision
    # ═════════════════════════════════════════════════════════════════════
    def _resolve(self, rnd):
        t_start = time.time()
        task = rnd.task
        avail = [r.rid for r in self.rovers.values()
                 if r.online and r.task is None]

        dec = AssignmentDecision()
        dec.task_id = task.id
        dec.stamp = self.get_clock().now().to_msg()

        scored = []
        for r in rnd.candidates:
            if r.rid not in rnd.paths:
                self._add_candidate(dec, r.rid, None, 'no path')
                continue
            parts, veto, sched = self._score(r, task, rnd.paths[r.rid], avail)
            if parts is None:
                self._add_candidate(dec, r.rid, None, veto)
                continue
            total = sum(parts.values())
            self._add_candidate(dec, r.rid, parts, '')
            scored.append((total, r, sched, parts))

        for rid, why in getattr(rnd, 'vetoes', {}).items():
            self._add_candidate(dec, rid, None, why)

        dec.decision_ms = (time.time() - t_start) * 1000.0

        if not scored:
            task.attempts += 1
            task.last_fail = self._now()
            dec.winner_id = ''
            dec.reason = 'no eligible rover'
            self.pub_decision.publish(dec)
            self.get_logger().warn(f'[{task.id}] no eligible rover this cycle')
            return

        scored.sort(key=lambda s: s[0])
        total, winner, sched, parts = scored[0]

        # Was the nearest rover passed over? That is the interesting case, so
        # say so explicitly in the log rather than burying it in the topic.
        nearest = min(scored, key=lambda s: s[3]['f1'])[1]
        note = ''
        if nearest.rid != winner.rid:
            gap = parts['f1'] - min(s[3]['f1'] for s in scored)
            note = (f'  (passed over {nearest.rid}, which is {gap:.0f} s closer '
                    f'— outweighed by other factors)')

        dec.winner_id = winner.rid
        dec.reason = (f'score {total:.1f} s effective' + note)
        self.pub_decision.publish(dec)

        self.get_logger().info(
            f'ASSIGN [{task.id}] -> {winner.rid}   score {total:.1f} s   '
            f'F1 {parts["f1"]:.0f} F2 {parts["f2"]:.0f} F3 {parts["f3"]:.0f} '
            f'F4 {parts["f4"]:.0f} F5 {parts["f5"]:.0f} F6 {parts["f6"]:.0f} '
            f'F7 {parts["f7"]:.0f}{note}')

        if winner.task is not None:                     # preemption
            self.get_logger().warn(
                f'{winner.rid}: preempting [{winner.task.id}] '
                f'(priority {winner.task.priority}) for [{task.id}] '
                f'(priority {task.priority})')
            self._release(winner, ok=False, requeue=True, preempted=True)

        winner.path = rnd.paths[winner.rid]
        winner.schedule = sched
        self._dispatch(winner, task)

    def _add_candidate(self, dec, rid, parts, veto):
        dec.candidate_ids.append(rid)
        z = parts or {}
        dec.score_total.append(sum(z.values()) if parts else BIG)
        for k, arr in (('f1', dec.f1_path_time), ('f2', dec.f2_turn_time),
                       ('f3', dec.f3_contention), ('f4', dec.f4_energy),
                       ('f5', dec.f5_reliability), ('f6', dec.f6_localization),
                       ('f7', dec.f7_coverage)):
            arr.append(float(z.get(k, 0.0)))
        dec.veto_reason.append(veto)

    # ═════════════════════════════════════════════════════════════════════
    #  Dispatch and completion
    # ═════════════════════════════════════════════════════════════════════
    def _pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        qx, qy, qz, qw = quat_from_yaw(float(yaw))
        p.pose.orientation.x, p.pose.orientation.y = qx, qy
        p.pose.orientation.z, p.pose.orientation.w = qz, qw
        return p

    def _dispatch(self, r, task, recharge=False):
        client = self.nav_clients[r.rid]
        if not client.server_is_ready():
            self.get_logger().warn(f'{r.rid}: Nav2 not ready, deferring [{task.id}]')
            return
        r.task = task
        r.path_t0 = self._now()
        task.assigned_to = r.rid
        task.recharge = recharge

        g = NavigateToPose.Goal()
        g.pose = self._pose(task.x, task.y, task.yaw)
        fut = client.send_goal_async(g)
        fut.add_done_callback(lambda f, rid=r.rid: self._on_nav_accepted(f, rid))

    def _on_nav_accepted(self, fut, rid):
        r = self.rovers[rid]
        try:
            handle = fut.result()
        except Exception as e:
            self.get_logger().error(f'{rid}: navigate goal rejected ({e})')
            self._release(r, ok=False, requeue=True)
            return
        if not handle.accepted:
            self.get_logger().error(f'{rid}: navigate goal rejected by Nav2')
            self._release(r, ok=False, requeue=True)
            return
        r.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, i=rid: self._on_nav_result(f, i))

    def _on_nav_result(self, fut, rid):
        r = self.rovers[rid]
        if r.task is None:
            return
        try:
            status = fut.result().status
        except Exception:
            status = GoalStatus.STATUS_ABORTED
        ok = (status == GoalStatus.STATUS_SUCCEEDED)
        tid = r.task.id
        # A docked rover is mechanically aligned to a surveyed pose, so this is
        # the one moment in the ward when the fleet knows exactly where it is.
        # Re-seeding here costs nothing and quietly repairs the slow AMCL drift
        # that a long shift accumulates.
        if ok and getattr(r.task, 'recharge', False) and r.task.owner == rid:
            self._seed_pose(r)
        if ok:
            self.completed += 1
            self.get_logger().info(
                f'DONE [{tid}] by {rid}  '
                f'(pack {r.tel.battery_wh:.0f} Wh, P(success) now '
                f'{self._peek_p(r, True):.2f})')
        else:
            self.get_logger().warn(f'FAILED [{tid}] on {rid} (nav status {status})')
        self._release(r, ok=ok, requeue=not ok)

    def _seed_pose(self, r):
        """Re-seed AMCL at the dock's surveyed pose with a tight covariance.

        Only ever called on a rover that actually reached its dock. It is NOT a
        recovery for a lost rover -- seeding a confident pose from a position
        the rover may not be at is worse than the uncertainty it replaces. That
        case goes through _relocalize instead.
        """
        dx, dy, dyaw = r.dock_pose
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(dx), float(dy)
        qx, qy, qz, qw = quat_from_yaw(float(dyaw))
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = qz, qw
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05          # 22 cm, the mechanical repeatability
        cov[35] = 0.02                  # of a docked rover
        m.pose.covariance = cov
        self.pub_initpose[r.rid].publish(m)
        self.get_logger().info(
            f'{r.rid}: docked at {r.dock_name}, AMCL re-seeded '
            f'({dx:.2f}, {dy:.2f})')

    def _peek_p(self, r, ok):
        d = float(self.G('reliability_decay'))
        a, b = r.alpha * d + (1.0 if ok else 0.0), r.beta * d + (0.0 if ok else 1.0)
        return a / (a + b)

    def _release(self, r, ok, requeue, preempted=False):
        task = r.task
        r.task = None
        r.goal_handle = None
        r.path = None
        r.schedule = []
        if task is None:
            return
        task.assigned_to = None
        # A preempted rover did nothing wrong, so its reliability is untouched.
        # Maintenance errands say nothing about whether a rover can do its job,
        # so they never move the reliability posterior.
        if not preempted and not getattr(task, 'recharge', False):
            r.record(ok, float(self.G('reliability_decay')))
        if ok:
            if task in self.queue:
                self.queue.remove(task)
            return
        task.attempts += 1
        task.last_fail = self._now()
        if task.owner is not None:
            # A failed sortie must NOT go back in the queue: the general
            # assignment path would hand rover3's docking errand to whichever
            # rover scored best, driving the wrong rover to the wrong dock and
            # re-seeding the wrong AMCL. _auto_dock and _relocalize re-raise it
            # on the owner next cycle if the condition persists.
            self.get_logger().info(
                f'[{task.id}] sortie failed; {task.owner} will retry when the '
                f'condition next holds')
            return
        if requeue and task.attempts < int(self.G('max_task_attempts')):
            self.reassigned += 1
            if task not in self.queue:
                self.queue.append(task)
            self.get_logger().info(
                f'[{task.id}] returned to the queue '
                f'(attempt {task.attempts}/{int(self.G("max_task_attempts"))})')
        else:
            if task in self.queue:
                self.queue.remove(task)
            if not preempted:
                self.get_logger().error(
                    f'[{task.id}] abandoned after {task.attempts} attempts')

    # ═════════════════════════════════════════════════════════════════════
    #  RViz
    # ═════════════════════════════════════════════════════════════════════
    def _publish_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        i = 0

        for name, (x, y, _yaw, dem) in self.stations.items():
            m = Marker()
            m.header.frame_id = 'map'; m.header.stamp = now
            m.ns = 'stations'; m.id = i; i += 1
            m.type = Marker.CYLINDER; m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.4 + dem * 0.5
            m.scale.z = 0.1
            m.color = ColorRGBA(r=0.2, g=0.6, b=0.9, a=0.55)
            ma.markers.append(m)

            t = Marker()
            t.header.frame_id = 'map'; t.header.stamp = now
            t.ns = 'station_labels'; t.id = i; i += 1
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, 0.7
            t.pose.orientation.w = 1.0
            t.scale.z = 0.45
            t.color = ColorRGBA(r=0.9, g=0.9, b=0.95, a=0.9)
            t.text = name
            ma.markers.append(t)

        for r in self.rovers.values():
            if not r.online:
                continue
            pct = 100.0 * r.tel.battery_wh / max(1.0, r.capacity_wh)
            lbl = Marker()
            lbl.header.frame_id = 'map'; lbl.header.stamp = now
            lbl.ns = 'rovers'; lbl.id = i; i += 1
            lbl.type = Marker.TEXT_VIEW_FACING; lbl.action = Marker.ADD
            lbl.pose.position.x, lbl.pose.position.y = r.pos
            lbl.pose.position.z = 1.4
            lbl.pose.orientation.w = 1.0
            lbl.scale.z = 0.5
            busy = r.task.id if r.task else 'idle'
            lbl.color = (ColorRGBA(r=1.0, g=0.85, b=0.2, a=1.0) if r.task
                         else ColorRGBA(r=0.5, g=0.9, b=0.5, a=1.0))
            lbl.text = (f'{r.rid}  {pct:.0f}%\n{busy}\n'
                        f'P={r.p_success:.2f}  s={r.tel.pose_sigma:.2f}')
            ma.markers.append(lbl)

            if r.path is not None:
                ln = Marker()
                ln.header.frame_id = 'map'; ln.header.stamp = now
                ln.ns = 'committed_paths'; ln.id = i; i += 1
                ln.type = Marker.LINE_STRIP; ln.action = Marker.ADD
                ln.scale.x = 0.08
                ln.pose.orientation.w = 1.0
                ln.color = ColorRGBA(r=1.0, g=0.6, b=0.1, a=0.8)
                ln.points = [Point(x=p.pose.position.x, y=p.pose.position.y, z=0.05)
                             for p in r.path.poses[::4]]
                ma.markers.append(ln)

        self.pub_markers.publish(ma)


def main():
    rclpy.init()
    node = FleetBrain()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
