#!/usr/bin/env python3
"""Build worlds/hospital_large.world (Gazebo Classic) from the extended
Ignition source world, then add the GForce-specific structures."""
import re, os

import sys
HERE = os.path.dirname(os.path.abspath(__file__)) if '__file__' in dir() else '.'
ROOT = os.path.dirname(HERE)
# The Ignition source world this was derived from. Kept as an argument because
# it lives outside the repo; pass a path to re-derive from a different source.
SRC = sys.argv[1] if len(sys.argv) > 1 else \
    '/home/ashwinder/Desktop/hospitaldelivry_bot/maps/usablefile'
DST = os.path.join(ROOT, 'worlds', 'hospital_large.world')

s = open(SRC).read()

# ---------------------------------------------------------------- 1. header
HEADER = '''<?xml version="1.0" ?>
<!-- GForce hospital ward - EXTENDED arena, Gazebo Classic 11.

     Arena is 33.2 m x 27.2 m (X -15.1..+18.1, Y -9.1..+18.1), roughly 2.2x
     the floor area of the original two-rover world.

     Converted from the Ignition/Fortress source: Classic 11 parses SDF only
     up to 1.7 and has no "ignored" physics type, so the header is SDF 1.6
     with an ODE engine. The ignition-gazebo-*-system plugins have no Classic
     equivalent (physics, sensors, scene broadcasting and user commands are
     built into gzserver) and are dropped. The ROS interface plugins
     (libgazebo_ros_init / _factory / _force_system) are injected by
     gazebo_ros's own gzserver launch file via -s arguments, so they must not
     be listed here either.

     Added for the four-rover centralised fleet: a surgical suite (SE), a
     pharmacy store (W), four structural columns, four charging docks that
     double as rover home poses, and floor zone markers. -->
<sdf version="1.6">
  <world name="hospital">

    <!-- ===== PHYSICS ===== -->
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
'''

# cut everything before the LIGHTING block and replace with our header
i = s.index('<!-- ===== LIGHTING')
s = HEADER + '\n    ' + s[i:]

# ------------------------------------------------- 2. drop commented actors
before = len(s)
s = re.sub(r'\n\s*<!-- ===== DYNAMIC ACTORS ===== -->.*?(?=\n\s*</world>)', '', s, flags=re.S)
assert '<actor' not in s, 'actor block survived'
print(f'  removed commented actor block ({before-len(s)} bytes)')

# ------------------------------- 3. drop the single old dock (4 replace it)
s = re.sub(r'\n\s*<!--[^\n]*[Cc]harging [Dd]ock[^\n]*-->', '', s)
s = re.sub(r'\n\s*<model name="charging_dock">.*?</model>', '', s, flags=re.S)
assert 'charging_dock' not in s

# -------------------------------------------- 4. restyle walls (new look)
# Slate-blue wainscot instead of the source world's flat white, so the
# extended arena is visually distinct from the world it was derived from.
def restyle(m):
    body = m.group(0)
    if not re.search(r'<model name="(wall_|corridor_|room1_|ot_north)', body):
        return body
    return re.sub(r'<material>.*?</material>',
                  '<material><ambient>0.72 0.76 0.80 1</ambient>'
                  '<diffuse>0.72 0.76 0.80 1</diffuse>'
                  '<specular>0.05 0.05 0.05 1</specular></material>',
                  body, flags=re.S)
s = re.sub(r'<model name="[^"]+">.*?</model>', restyle, s, flags=re.S)

# ------ 4b. raise floor obstacles into the 0.75 m scan plane -------------
# The rovers carry the lidar at z~0.75 (0.60 above a base_link that sits at
# the 0.15 m wheel radius). Anything shorter than that is invisible to the
# scanner, so it never lands in the occupancy grid and the global planner
# happily routes straight through it. Rather than hand the planner a map the
# lidar disagrees with -- which would bias AMCL -- the few floor-standing
# obstacles are given realistic full heights: hospital beds at their raised
# working height, supply crates stacked, hazard sign and laundry sack on
# stands. Everything else that fails the plane test is either wall-mounted,
# resting on furniture that IS visible, or a drive-over charging plate.
RESIZE = {
    # name          : (new pose z, new size)
    'supply_box_1'  : (0.50, '0.6 0.6 1.0'),
    'supply_box_2'  : (0.50, '0.6 0.6 1.0'),
    'supply_box_3'  : (0.50, '0.6 0.6 1.0'),
    'bed_room1'     : (0.475, '0.95 2.1 0.95'),
    'bed_room2'     : (0.475, '0.95 2.1 0.95'),
    'pillow_room1'  : (1.00, '0.6 0.4 0.1'),
    'pillow_room2'  : (1.00, '0.6 0.4 0.1'),
    'wet_floor_sign': (0.45, '0.5 0.35 0.9'),
    'laundry_bag'   : (0.45, '0.5 0.5 0.9'),
}
def resize(m):
    body = m.group(0)
    nm = re.match(r'<model name="([^"]+)">', body).group(1)
    if nm not in RESIZE:
        return body
    z, size = RESIZE[nm]
    body = re.sub(r'(<pose>\s*\S+\s+\S+\s+)\S+', lambda k: k.group(1) + str(z), body, count=1)
    body = re.sub(r'<size>[^<]+</size>', f'<size>{size}</size>', body)
    return body
s = re.sub(r'<model name="[^"]+">.*?</model>', resize, s, flags=re.S)

# ------ 4c. move source furniture out of the doorways this world adds -------
# The pharmacy is new; laundry_cart is not. The cart sat 1.10 m east of where
# the new pharmacy door was cut, and a rover with footprint padding needs
# 0.97 m. It fitted geometrically -- which is why a pure connectivity check
# passed it -- but Nav2 inflates obstacles to 0.55 m, so the gap was lethal
# cost end to end. Rovers wedged there, thrashed, and their AMCL diverged.
# Moving the cart into open floor is the fix; tools/build_map.py --check now
# also fails any corridor this marginal.
MOVE = {
    'laundry_cart': (-4.00, 0.00),
}
def relocate(m):
    body = m.group(0)
    nm = re.match(r'<model name="([^"]+)">', body).group(1)
    if nm not in MOVE:
        return body
    x, y = MOVE[nm]
    return re.sub(r'<pose>\s*\S+\s+\S+(\s+.*?)</pose>',
                  lambda k: f'<pose>{x} {y}{k.group(1)}</pose>', body, count=1)
s = re.sub(r'<model name="[^"]+">.*?</model>', relocate, s, flags=re.S)

# ---------------------------------------------------------- 5. new models
W = 3.0                                   # standard wall height
WALL   = (0.72, 0.76, 0.80)
STEEL  = (0.55, 0.58, 0.62)
CREAM  = (0.82, 0.78, 0.68)
GREEN  = (0.35, 0.55, 0.45)
TEAL   = (0.30, 0.60, 0.62)
WHITE  = (0.90, 0.90, 0.92)
DOCK   = (0.15, 0.65, 0.35)

boxes = [
    # -------- Surgical wing, south-east (x 6..18, y -9..-3) --------------
    ('sw_wall_west',      6.00, -4.50, W/2,   0.20,  9.00, W, WALL),
    ('sw_wall_div_w',     9.00, -3.00, W/2,   6.00,  0.20, W, WALL),
    ('sw_wall_div_e',    16.55, -3.00, W/2,   3.10,  0.20, W, WALL),
    ('ot_table',         12.00, -6.00, 0.425, 0.90,  2.20, 0.85, STEEL),
    ('ot_lamp_head',     12.00, -5.60, 2.35,  1.10,  1.10, 0.18, WHITE),
    ('anesthesia_cart',   9.80, -6.00, 0.60,  0.60,  0.50, 1.20, TEAL),
    ('instrument_tray',  14.20, -6.00, 0.50,  0.90,  0.60, 1.00, STEEL),
    ('scrub_sink',       16.60, -8.00, 0.525, 1.60,  0.60, 1.05, WHITE),
    ('sterile_cabinet',   7.20, -8.00, 1.00,  1.20,  0.50, 2.00, CREAM),
    # -------- Pharmacy store, west (x -15..-9, y -3..+3) ----------------
    ('ph_wall_east_s',   -9.00, -2.00, W/2,   0.20,  2.00, W, WALL),
    ('ph_wall_east_n',   -9.00,  2.00, W/2,   0.20,  2.00, W, WALL),
    ('ph_wall_south',   -12.05, -3.00, W/2,   6.10,  0.20, W, WALL),
    ('ph_wall_north',   -12.05,  3.00, W/2,   6.10,  0.20, W, WALL),
    ('ph_shelf_a',      -12.50,  2.25, 1.00,  4.00,  0.50, 2.00, CREAM),
    ('ph_shelf_b',      -12.50, -2.25, 1.00,  4.00,  0.50, 2.00, CREAM),
    ('ph_counter',      -14.50,  0.00, 0.525, 0.60,  1.60, 1.05, GREEN),
    # -------- Signage (above the 0.75 m scan plane) ---------------------
    ('sign_surgical',    10.50, -2.88, 1.60,  0.60,  0.02, 0.20, GREEN),
    ('sign_pharmacy',    -8.88,  2.00, 1.60,  0.02,  0.60, 0.20, GREEN),
]

cyls = [
    ('column_1', -12.00, -5.40, 1.50, 0.25, W,   STEEL),
    ('column_2',  -6.00, -5.40, 1.50, 0.25, W,   STEEL),
    ('column_3',   0.00, -5.40, 1.50, 0.25, W,   STEEL),
    ('column_4',  -6.00,  2.40, 1.50, 0.25, W,   STEEL),
    ('ot_lamp_post', 12.00, -5.60, 1.30, 0.08, 2.60, STEEL),
]

# Charging docks: 0.04 m plates, well below the 0.75 m scan plane, so they
# are landmarks for the operator and home poses for the fleet without
# showing up as obstacles in the map.
DOCKS = {'dock_alpha':   (-13.00, -7.50),
         'dock_bravo':   ( 15.50,   2.00),
         'dock_charlie': (-13.00,   7.50),
         'dock_delta':   ( 12.00,   7.50)}
for n, (x, y) in DOCKS.items():
    boxes.append((n, x, y, 0.02, 0.90, 0.90, 0.04, DOCK))

# Visual-only floor zone markers (no <collision>) - purely a look change.
zones = [
    ('zone_surgical',  12.00, -6.00, 12.00,  6.00, (0.45, 0.28, 0.30)),
    ('zone_pharmacy', -12.05,  0.00,  6.00,  6.00, (0.28, 0.42, 0.34)),
    ('zone_ward_a',     0.00, 11.25,  9.00, 13.50, (0.30, 0.36, 0.48)),
    ('zone_supply',   -10.00, 11.25, 10.00, 13.50, (0.44, 0.40, 0.30)),
]

def mat(c):
    r, g, b = c
    return (f'<material><ambient>{r} {g} {b} 1</ambient>'
            f'<diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>0.05 0.05 0.05 1</specular></material>')

out = ['\n    <!-- ===== GFORCE EXTENSIONS (4-rover centralised fleet) ===== -->\n']
for n, x, y, z, sx, sy, sz, c in boxes:
    g = f'<box><size>{sx} {sy} {sz}</size></box>'
    out.append(f'''    <model name="{n}">
      <static>true</static><pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry>{g}</geometry></collision>
        <visual name="vis"><geometry>{g}</geometry>
          {mat(c)}
        </visual>
      </link>
    </model>
''')
for n, x, y, z, r, l, c in cyls:
    g = f'<cylinder><radius>{r}</radius><length>{l}</length></cylinder>'
    out.append(f'''    <model name="{n}">
      <static>true</static><pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry>{g}</geometry></collision>
        <visual name="vis"><geometry>{g}</geometry>
          {mat(c)}
        </visual>
      </link>
    </model>
''')
for n, x, y, sx, sy, c in zones:
    g = f'<box><size>{sx} {sy} 0.01</size></box>'
    out.append(f'''    <model name="{n}">
      <static>true</static><pose>{x} {y} 0.005 0 0 0</pose>
      <link name="link">
        <visual name="vis"><geometry>{g}</geometry>
          {mat(c)}
        </visual>
      </link>
    </model>
''')

s = s.replace('\n  </world>', '\n' + ''.join(out) + '\n  </world>')

os.makedirs(os.path.dirname(DST), exist_ok=True)
open(DST, 'w').write(s)
print(f'  wrote {DST}  ({len(s)} bytes)')
print(f'  models: {len(re.findall(chr(60)+chr(109)+"odel name=", s))}')
