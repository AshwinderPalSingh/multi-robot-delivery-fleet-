#!/usr/bin/env python3
"""
rover_link — the boundary between the central brain and one rover.

This node is the whole point of the architecture. Everything above it (task
queue, assignment policy, planners, costmaps, localisation) runs on the
central brain. Everything below it is a motor controller. rover_link is the
single place the two meet, and it presents the same interface in both
directions regardless of which side is real:

    mode:=sim        the "rover" is a Gazebo model. Velocity commands are
                     republished on <ns>/wheel_cmd; telemetry is synthesised
                     from Gazebo odometry, AMCL and a simulated battery.

    mode:=hardware   the rover is an ESP32 over Wi-Fi. The same velocity pair
                     is packed into the 8-byte downlink frame in
                     docs/HARDWARE.md and sent by UDP; telemetry is decoded
                     from the 16-byte uplink frame and fused with the
                     brain-side AMCL estimate.

Because the brain talks only in RoverCommand and RoverTelemetry, porting the
fleet from simulation to hardware changes this node's mode parameter and
nothing else. No planner, no costmap and no decision logic ever moves onto
the rover.
"""

import math
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

from hospital_robot_description.msg import RoverCommand, RoverTelemetry


# ── wire format ────────────────────────────────────────────────────────────
# Kept byte-identical to the ESP32 firmware in firmware/rover_esp32/.
DOWN_MAGIC = 0xA5
UP_MAGIC   = 0x5A
DOWN_FMT   = '<BBhhBB'      # 8 bytes:  magic, seq, v_mm_s, w_mrad_s, mode, crc8
UP_FMT     = '<BBiiHhBB'    # 16 bytes: magic, seq, ticks_l, ticks_r, vbat_mv,
                            # gyro_z_mrad, status, crc8


def crc8(data: bytes) -> int:
    """Dallas/Maxim CRC-8, poly 0x31. Cheap enough for an ESP32 ISR."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RoverLink(Node):

    def __init__(self):
        super().__init__('rover_link')

        self.declare_parameter('rover_id', 'rover1')
        self.declare_parameter('mode', 'sim')             # sim | hardware
        self.declare_parameter('esp_endpoint', '')        # host:port, hardware only
        # One port per rover: four rover_link nodes share the brain, so a
        # single hard-coded port would make three of them fail to bind.
        self.declare_parameter('listen_port', 9101)
        self.declare_parameter('battery_wh', 480.0)
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('override_timeout_s', 0.4)
        # One timer drives both directions: it emits the velocity command and
        # publishes telemetry. 20 Hz matches the downlink rate documented in
        # docs/HARDWARE.md and stays comfortably inside the 300 ms watchdog
        # the ESP32 enforces even if a few frames are dropped.
        self.declare_parameter('tick_rate_hz', 20.0)
        # Energy model — the same constants the brain scores with, so its
        # predictions and the pack it is predicting about cannot drift apart.
        self.declare_parameter('energy_wh_per_m', 0.12)
        self.declare_parameter('energy_wh_per_rad', 0.05)
        self.declare_parameter('energy_idle_w', 15.0)
        self.declare_parameter('sim_drain_multiplier', 12.0)
        self.declare_parameter('dock_x', 0.0)
        self.declare_parameter('dock_y', 0.0)
        self.declare_parameter('dock_radius', 0.8)
        self.declare_parameter('charge_w', 900.0)

        g = lambda n: self.get_parameter(n).value
        self.rid       = g('rover_id')
        self.mode      = g('mode')
        self.cap_wh    = float(g('battery_wh'))
        self.cmd_to    = float(g('command_timeout_s'))
        self.ovr_to    = float(g('override_timeout_s'))
        self.wh_per_m  = float(g('energy_wh_per_m'))
        self.wh_per_rad= float(g('energy_wh_per_rad'))
        self.idle_w    = float(g('energy_idle_w'))
        self.drain_mul = float(g('sim_drain_multiplier'))
        self.dock      = (float(g('dock_x')), float(g('dock_y')))
        self.dock_r    = float(g('dock_radius'))
        self.charge_w  = float(g('charge_w'))

        # ── live state ────────────────────────────────────────────────────
        self.batt_wh   = self.cap_wh
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.pose_sigma = 0.5              # until AMCL converges
        self.odom_total = 0.0
        self.seq = 0
        self.last_cmd_t = 0.0
        self.last_cmd = (0.0, 0.0)
        self.last_ovr_t = 0.0
        self.last_ovr = (0.0, 0.0)
        self.last_uplink_t = None
        self.rtt_ms = 0
        self.estop = False
        self.have_odom = False

        ns = f'/{self.rid}'
        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)

        # ── downlink: Nav2 velocity in, motor command out ─────────────────
        self.create_subscription(Twist, f'{ns}/cmd_vel', self._on_cmd_vel, 10)
        # Direct brain control, and it outranks Nav2. /cmd_vel is shared by the
        # controller, the behaviour server and the velocity smoother, so a
        # brain-issued motion posted there would fight their zero-publishes and
        # stutter. Recovery rotations, docking nudges and e-stops come down
        # this channel instead and win while they are fresh.
        self.create_subscription(Twist, f'{ns}/override_cmd', self._on_override, 10)
        self.pub_cmd = self.create_publisher(RoverCommand, f'{ns}/command', 10)
        self.pub_wheel = self.create_publisher(Twist, f'{ns}/wheel_cmd', 10)

        # ── uplink: telemetry to the brain ────────────────────────────────
        self.pub_tel = self.create_publisher(RoverTelemetry, f'{ns}/telemetry', 10)

        if self.mode == 'sim':
            self.create_subscription(Odometry, f'{ns}/odom', self._on_odom, sensor_qos)
        else:
            host, _, port = g('esp_endpoint').partition(':')
            self.tx_addr = (host, int(port or 9001))
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(('0.0.0.0', int(g('listen_port'))))
            self.sock.settimeout(0.2)
            threading.Thread(target=self._uplink_loop, daemon=True).start()
            self.get_logger().info(f'{self.rid}: UDP link to {self.tx_addr}')

        # AMCL runs on the brain in BOTH modes — the rover never localises
        # itself. This is the pose the whole fleet is scored against.
        amcl_qos = QoSProfile(depth=5,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseWithCovarianceStamped, f'{ns}/amcl_pose',
                                 self._on_amcl, amcl_qos)

        period = 1.0 / float(g('tick_rate_hz'))
        self.create_timer(period, self._tick)
        self._t_prev = self._now()

        self.get_logger().info(
            f'rover_link [{self.rid}] mode={self.mode} pack={self.cap_wh:.0f} Wh')

    # ── clock ─────────────────────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # ── downlink ──────────────────────────────────────────────────────────
    def _on_cmd_vel(self, msg: Twist):
        self.last_cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd_t = self._now()

    def _on_override(self, msg: Twist):
        self.last_ovr = (msg.linear.x, msg.angular.z)
        self.last_ovr_t = self._now()

    def _emit(self, v: float, w: float, mode: int):
        self.seq = (self.seq + 1) & 0xFFFF

        cmd = RoverCommand()
        cmd.rover_id = self.rid
        cmd.mode = mode
        cmd.v_mps = v
        cmd.w_rps = w
        cmd.seq = self.seq
        cmd.stamp = self.get_clock().now().to_msg()
        self.pub_cmd.publish(cmd)

        if self.mode == 'sim':
            t = Twist()
            t.linear.x = v
            t.angular.z = w
            self.pub_wheel.publish(t)
        else:
            body = struct.pack('<BBhhB', DOWN_MAGIC, self.seq & 0xFF,
                               int(v * 1000), int(w * 1000), mode)
            self.sock.sendto(body + bytes([crc8(body)]), self.tx_addr)

    # ── uplink (hardware) ─────────────────────────────────────────────────
    def _uplink_loop(self):
        while rclpy.ok():
            try:
                pkt, _ = self.sock.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(pkt) != struct.calcsize(UP_FMT) or pkt[0] != UP_MAGIC:
                continue
            if crc8(pkt[:-1]) != pkt[-1]:
                continue
            _, _, tl, tr, vbat, gyro, status = struct.unpack(UP_FMT, pkt)[:7]
            # Wheel odometry is integrated on the brain, not the rover: the
            # ESP32 sends raw tick counts and nothing else.
            self._integrate_ticks(tl, tr)
            self.batt_wh = self._wh_from_volts(vbat / 1000.0)
            self.estop = bool(status & 0x01)
            self.last_uplink_t = self._now()

    def _wh_from_volts(self, v: float) -> float:
        """Crude 6S Li-ion pack curve: 25.2 V full, 19.8 V empty."""
        frac = max(0.0, min(1.0, (v - 19.8) / (25.2 - 19.8)))
        return frac * self.cap_wh

    def _integrate_ticks(self, tl: int, tr: int):
        prev = getattr(self, '_ticks', None)
        self._ticks = (tl, tr)
        if prev is None:
            return
        TPR, R, B = 1200.0, 0.150, 0.720      # ticks/rev, wheel radius, track
        dl = (tl - prev[0]) / TPR * 2 * math.pi * R
        dr = (tr - prev[1]) / TPR * 2 * math.pi * R
        d, dth = (dl + dr) / 2.0, (dr - dl) / B
        self.x += d * math.cos(self.yaw + dth / 2.0)
        self.y += d * math.sin(self.yaw + dth / 2.0)
        self.yaw = math.atan2(math.sin(self.yaw + dth), math.cos(self.yaw + dth))
        self.odom_total += abs(d)

    # ── uplink (sim) ──────────────────────────────────────────────────────
    def _on_odom(self, msg: Odometry):
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z
        self.have_odom = True

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        self.x, self.y, self.yaw = p.position.x, p.position.y, yaw_of(p.orientation)
        c = msg.pose.covariance
        # sqrt of the xy covariance trace: one number for "how well does the
        # fleet know where this rover is". Factor F6 scores on it and vetoes
        # a rover that has lost the plot entirely.
        self.pose_sigma = math.sqrt(max(0.0, c[0] + c[7]))

    # ── periodic ──────────────────────────────────────────────────────────
    def _tick(self):
        now = self._now()
        dt = max(1e-3, min(1.0, now - self._t_prev))
        self._t_prev = now

        # Watchdog. A rover that stops hearing from the brain must stop
        # moving — this is the same 500 ms rule the ESP32 enforces locally,
        # duplicated here so the sim cannot mask a missing timeout.
        stale = (now - self.last_cmd_t) > self.cmd_to
        override = (now - self.last_ovr_t) <= self.ovr_to
        if self.estop:
            v, w, mode = 0.0, 0.0, RoverCommand.MODE_ESTOP
        elif override:
            v, w = self.last_ovr
            mode = RoverCommand.MODE_VELOCITY
        elif stale:
            v, w, mode = 0.0, 0.0, RoverCommand.MODE_STOP
        else:
            v, w = self.last_cmd
            mode = RoverCommand.MODE_VELOCITY
        self._emit(v, w, mode)

        if self.mode == 'sim':
            self._sim_battery(dt)
            self.odom_total += abs(self.v) * dt

        self._publish_telemetry(now, stale)

    def _sim_battery(self, dt: float):
        on_dock = math.hypot(self.x - self.dock[0], self.y - self.dock[1]) < self.dock_r
        if on_dock and abs(self.v) < 0.02 and abs(self.w) < 0.02:
            self.batt_wh = min(self.cap_wh, self.batt_wh + self.charge_w * dt / 3600.0)
            return
        drain = (self.wh_per_m * abs(self.v) * dt
                 + self.wh_per_rad * abs(self.w) * dt) * self.drain_mul
        drain += self.idle_w * dt / 3600.0
        self.batt_wh = max(0.0, self.batt_wh - drain)

    def _publish_telemetry(self, now: float, stale_cmd: bool):
        t = RoverTelemetry()
        t.rover_id = self.rid

        if self.mode == 'sim':
            t.link_state = RoverTelemetry.LINK_OK
            t.comms_rtt_ms = 2
        else:
            age = 1e9 if self.last_uplink_t is None else now - self.last_uplink_t
            t.link_state = (RoverTelemetry.LINK_OK if age < 0.5 else
                            RoverTelemetry.LINK_STALE if age < 3.0 else
                            RoverTelemetry.LINK_OFFLINE)
            t.comms_rtt_ms = int(min(9999, age * 1000))

        moving = abs(self.v) > 0.03 or abs(self.w) > 0.05
        on_dock = math.hypot(self.x - self.dock[0], self.y - self.dock[1]) < self.dock_r
        if self.estop:
            t.mission_state = RoverTelemetry.STATE_FAULT
        elif on_dock and not moving:
            t.mission_state = RoverTelemetry.STATE_CHARGING
        elif moving:
            t.mission_state = RoverTelemetry.STATE_EN_ROUTE
        else:
            t.mission_state = RoverTelemetry.STATE_IDLE

        t.x, t.y, t.yaw = self.x, self.y, self.yaw
        t.pose_sigma = self.pose_sigma
        t.v, t.w = self.v, self.w
        t.battery_wh = self.batt_wh
        t.battery_volts = 19.8 + (self.batt_wh / self.cap_wh) * (25.2 - 19.8)
        t.motor_current_a = abs(self.v) * 6.0 + abs(self.w) * 2.0
        t.active_task_id = ''
        t.odom_total_m = self.odom_total
        t.stamp = self.get_clock().now().to_msg()
        self.pub_tel.publish(t)


def main():
    rclpy.init()
    node = RoverLink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
