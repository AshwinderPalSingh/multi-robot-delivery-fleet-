# GForce — Hardware Design

This document is the bridge between the simulation in this repository and the
physical fleet. The simulation was built to this design, not the other way
around: `rover_link` already speaks the protocol below, and switching it from
`mode:=sim` to `mode:=hardware` is the whole port.

---

## 1. The division of labour

The single decision in this architecture is **where the computation lives**,
and the answer is *all of it, in one place*.

```
┌──────────────────────────────────────────────────────────────────┐
│  CENTRAL BRAIN — one static box (Raspberry Pi 5 / Jetson Orin)   │
│                                                                   │
│   fleet_brain          task queue, 7-factor assignment policy,    │
│                        preemption, reliability history            │
│   map_server           one shared occupancy grid                  │
│   Nav2 x4              AMCL, global planner, DWB controller,      │
│                        costmaps, recovery behaviours              │
│   rover_link x4        protocol adapter — the ONLY thing that     │
│                        ever commands a motor                      │
│   task_console         where ward staff submit work               │
└───────────────────────────┬──────────────────────────────────────┘
                            │  Wi-Fi (802.11n, 5 GHz, dedicated AP)
        ┌───────────────────┼───────────────────┬──────────────────┐
        │                   │                   │                  │
   ┌────┴─────┐       ┌─────┴────┐       ┌──────┴───┐       ┌──────┴───┐
   │ rover1   │       │ rover2   │       │ rover3   │       │ rover4   │
   │ ESP32    │       │ ESP32    │       │ ESP32    │       │ ESP32    │
   │ + BTS7960│       │          │       │          │       │          │
   │ + encoders       │          │       │          │       │          │
   │ + RPLIDAR│       │          │       │          │       │          │
   └──────────┘       └──────────┘       └──────────┘       └──────────┘
```

A rover runs **one control loop**: wheel velocity PID on encoder feedback. It
holds no map, no plan, no goal, and no knowledge that other rovers exist.

What that buys:

| | |
|---|---|
| **Retuning** | Edit `fleet_brain.yaml`, restart one node. No reflashing four rovers. |
| **Consistency** | Four rovers cannot hold four conflicting beliefs about the ward, because they hold none. |
| **Cost per rover** | An ESP32 and a motor driver. The expensive compute is bought once. |
| **Debugging** | Every decision, every path and every telemetry frame is on one machine, in one log, on one clock. |
| **Better decisions** | Factors F3, F5 and F7 in the assignment policy are *only* computable centrally — see §7. |

And what it costs — stated honestly:

| | |
|---|---|
| **Single point of failure** | The brain goes down, the fleet stops. Mitigation in §8. |
| **Link dependence** | A rover out of Wi-Fi range stops within 300 ms. It cannot finish its errand alone. |
| **Compute ceiling** | Four Nav2 stacks on one box is the real constraint. See §2. |
| **Latency floor** | Control is closed over Wi-Fi, so jitter matters. See §3. |

---

## 2. Compute budget — the honest version

The heavy load is **not** `fleet_brain`. Scoring four candidates across seven
factors takes single-digit milliseconds, and it runs once per assignment, not
per control cycle. The load is **four concurrent Nav2 stacks**.

Per rover, roughly:

| component | CPU |
|---|---|
| AMCL (2000 particles, 360-beam scan, 10 Hz) | ~12 % of one core |
| global costmap 33 × 27 m @ 5 cm, 0.5 Hz | ~6 % |
| local costmap 5 × 5 m rolling, 5 Hz | ~10 % |
| DWB controller, 20 Hz | ~18 % |
| planner + BT + lifecycle | ~8 % |
| **total** | **~0.55 core** |

Four rovers ≈ **2.2 cores sustained**, plus map_server, four rover_links, TF
and DDS traffic.

| board | verdict |
|---|---|
| Raspberry Pi 4 (4×A72 @ 1.8 GHz) | **No.** Two rovers at best, and thermal throttling eats the margin. |
| Raspberry Pi 5 (4×A76 @ 2.4 GHz) | **Marginal.** Four rovers fit with the particle counts already reduced to 2000/800 in `config/nav2_params_rover*.yaml`. Needs active cooling. No headroom for a fifth rover. |
| Jetson Orin Nano (6×A78AE) | **Recommended.** Comfortable at four, room to grow, and the GPU is free for a camera stage later. |

If the fleet has to grow past four on a Pi 5, the lever is AMCL: drop to
1000/400 particles and 5 Hz, or replace it with wheel odometry plus fiducial
markers at known ward locations.

**Do not** move Nav2 onto the rovers to solve this. That converts the system
back into four independent robots and destroys factors F3, F5 and F7 —
which is the entire reason the architecture is centralised.

---

## 3. Network

Dedicated AP, 5 GHz, **not** the hospital's guest network. The brain is the
gateway at `192.168.4.1`.

| | rate | payload | bandwidth |
|---|---|---|---|
| downlink, brain → rover | 20 Hz | 8 B | 160 B/s |
| uplink, rover → brain | 50 Hz | 16 B | 800 B/s |
| lidar, rover → brain | 10 Hz | ~1.4 kB | 14 kB/s |
| **per rover** | | | **~15 kB/s** |
| **fleet of four** | | | **~60 kB/s** |

Bandwidth is a non-issue. **Latency jitter is the real constraint** — the
velocity loop is closed over the air, so a 200 ms stall is a rover that
overshoots. Practical measures:

- `WiFi.setSleep(false)` on the ESP32. Power save adds tens of milliseconds.
- UDP, never TCP. A late velocity command is worthless; retransmitting it is
  actively harmful, which is why the firmware drains its socket and keeps only
  the newest frame.
- Fixed DHCP leases so `esp_endpoint` in `fleet_brain.yaml` stays valid.
- One AP per ward. Roaming between APs mid-corridor causes multi-second
  dropouts, and the watchdog will (correctly) stop the rover.

---

## 4. Wire protocol

Byte-identical in `scripts/rover_link.py` and
`firmware/rover_esp32/rover_esp32.ino`. Little-endian, CRC-8 Dallas/Maxim
(poly `0x31`) over all preceding bytes.

### Downlink — brain to rover, 8 bytes, 20 Hz

| off | type | field | notes |
|---|---|---|---|
| 0 | `u8` | magic | `0xA5` |
| 1 | `u8` | seq | wraps; for loss accounting |
| 2 | `i16` | v_mm_s | forward body velocity, mm/s |
| 4 | `i16` | w_mrad_s | yaw rate, mrad/s |
| 6 | `u8` | mode | 0 stop, 1 velocity, 2 dock, 3 e-stop |
| 7 | `u8` | crc8 | |

### Uplink — rover to brain, 16 bytes, 50 Hz

| off | type | field | notes |
|---|---|---|---|
| 0 | `u8` | magic | `0x5A` |
| 1 | `u8` | seq | |
| 2 | `i32` | ticks_l | **raw** cumulative encoder count |
| 6 | `i32` | ticks_r | |
| 10 | `u16` | vbat_mv | pack millivolts |
| 12 | `i16` | gyro_z_mrad | yaw rate, if an IMU is fitted |
| 14 | `u8` | status | bit0 e-stop, bit1 stall, bit2 undervolt |
| 15 | `u8` | crc8 | |

Note that the rover sends **raw tick counts**, not a pose. Odometry is
integrated on the brain. The rover has no opinion about where it is.

### Command priority

`rover_link` accepts two ROS-side inputs and resolves them before anything is
packed into a downlink frame:

| priority | source | used when |
|---|---|---|
| 1 | e-stop (hardware or `status` bit0) | always wins; zero velocity |
| 2 | `/roverX/override_cmd` | a frame arrived in the last 400 ms |
| 3 | `/roverX/cmd_vel` (Nav2) | a frame arrived in the last 500 ms |
| 4 | — | nothing fresh: zero velocity, `MODE_STOP` |

Only one velocity pair ever leaves for the rover, so the ESP32 never has to
arbitrate between sources — which is the point of keeping arbitration on the
brain.

### Watchdog

Both ends hold the same rule, deliberately duplicated so neither can mask the
other's failure:

- **Rover**: no valid downlink frame for **300 ms** → motors off, integrators
  cleared. Also on Wi-Fi disassociation.
- **Brain**: no uplink for **500 ms** → `LINK_STALE`; 3 s → `LINK_OFFLINE`
  and the rover is vetoed from all assignments; mid-mission, the task is
  requeued for another rover.

---

## 5. Rover bill of materials

Per rover:

| part | notes |
|---|---|
| ESP32-WROOM-32 | 2 free hardware UARTs, enough PWM channels, adequate Wi-Fi |
| 2 × BTS7960 43 A half-bridge | or any PWM+DIR driver matched to the motors |
| 2 × geared DC motor, 24 V, with quadrature encoder | 1200 counts/rev after gearing |
| RPLIDAR A3 (25 m) | UART2, 256000 baud. **Not an A1/A2** — a 12 m scanner leaves the 33 m main hall under-observed and AMCL diverges crossing it; see README. |
| 6S Li-ion pack, 24 V nominal | capacity per `battery_wh` in `fleet_brain.yaml` |
| INA219 or 100k/15k divider | pack voltage sense into GPIO4 |
| MPU6050 | optional; improves rotation odometry |
| NC mushroom e-stop | to GPIO13, pulled up |
| DC-DC 24 V → 5 V, 3 A | ESP32 + lidar |

### Pin map (matches the firmware)

| function | pin |
|---|---|
| left RPWM / LPWM / EN | 25 / 26 / 27 |
| right RPWM / LPWM / EN | 32 / 33 / 14 |
| left encoder A / B | 34 / 35 |
| right encoder A / B | 36 / 39 |
| pack voltage sense | 4 |
| e-stop (NC to GND) | 13 |
| RPLIDAR RX / TX | 16 / 17 (UART2, 256000 baud) |

GPIO 34–39 are input-only on the ESP32 — correct for encoders, and they must
not be used for the driver outputs.

### Lidar path

The RPLIDAR's serial stream is forwarded verbatim over UDP; the brain
reconstructs `sensor_msgs/LaserScan`. The ESP32 does **not** parse scans. This
keeps the rover dumb and puts the scan on the same machine as the costmap that
consumes it — no clock skew between scan and map.

---

## 6. Bring-up

1. **Flash.** Set `ROVER_INDEX` (1–4) and the Wi-Fi credentials in
   `rover_esp32.ino`. That index is the only rover-specific constant in the
   firmware; everything else comes from the brain at runtime.
2. **Fix the IPs.** DHCP reservations matching `esp_endpoint` in
   `fleet_brain.yaml` (`192.168.4.11`–`.14`).
3. **Check the encoders.** Push each rover forward by hand; both tick counts
   must *increase*. If one decreases, swap its A/B pair — do not fix it in
   software, or the PID sign will be wrong under load.
4. **Tune the PID.** Command a step of 0.2 m/s with the wheels off the ground,
   log `ticks`, adjust `KP`/`KI`/`KD`. Then repeat under load — a heavy
   chassis needs materially more integral term.
5. **Calibrate the energy model.** Drive a known loop (say 100 m) at cruise,
   log pack Wh consumed, and set `energy_wh_per_m`. Repeat with pure rotation
   for `energy_wh_per_rad`. **Do this before trusting factor F4** — an
   uncalibrated energy model will either veto everything or strand a rover.
6. **Set `energy_scale: 1.0`** in the brain. The simulation runs it at 12.0 to
   exercise F4 in minutes instead of hours; on hardware that would make the
   brain refuse missions it could easily afford.
7. **Map the ward.** `slam_toolbox` with one rover under teleop, then
   `map_saver_cli`. Mount the lidar at the same height on every rover — the
   map is shared, and a rover scanning at a different height sees a different
   ward.
8. **Switch the links.** `mode:=hardware` in `fleet_full.launch.py`, and drop
   the Gazebo and spawn actions.

---

## 7. Why the policy needs a centre

Three of the seven assignment factors are not implementable on a rover, at
any effort, because the information does not exist there:

- **F3 contention forecast** compares a candidate path against *every other
  rover's committed path*, in space and time, weighted by how narrow the
  corridor is at each conflict. A rover bidding for itself does not know where
  the others intend to go. Decentralised fleets discover the conflict by
  driving into it.
- **F5 reliability prior** is a Beta posterior over each rover's mission
  history. A rover could track its own failures, but it cannot compare itself
  to its peers, and it has every incentive not to.
- **F7 fleet readiness** asks how well the *whole ward* is still covered after
  this rover is committed. It is a property of the fleet, not of any rover.

This is the substantive argument for the architecture. The centralisation is
not a convenience — it is what makes the policy possible.

---

## 8. Failure modes

| failure | detection | response |
|---|---|---|
| Rover leaves Wi-Fi range | no uplink 300 ms (rover) / 3 s (brain) | rover stops; brain vetoes it and requeues its task |
| Wheel jams | commanded > 0.10 m/s, measured < 0.02 m/s | `status` bit1; F5 quietly de-prioritises the rover |
| Pack sags | `vbat_mv` below 19.8 V | `status` bit2; F4 vetoes, brain sends it to dock |
| AMCL diverges | `pose_sigma` > 1.2 m | rover vetoed; particles scattered globally and the rover rotates in place to re-converge; after 3 attempts it asks for a human |
| Two rovers meet in a corridor | should not happen — F3 prevents it | Nav2 local costmap handles it; the event is logged as an F3 miss and is the signal to raise `w3_contention` |
| **Brain dies** | rovers stop within 300 ms | see below |

The brain being a single point of failure is real and is the price of the
architecture. What makes it acceptable is that the failure is **safe**: every
rover stops within 300 ms of losing the link, which is the correct behaviour
for a machine in a hospital corridor. It is not a fleet that keeps moving with
stale plans.

For a production deployment the mitigation is a warm standby: a second Pi
subscribed to the same telemetry, holding the same fleet state, promoted by
keepalive. Rovers would need only a second endpoint to accept commands from.
That is not implemented here and should not be claimed as such.
