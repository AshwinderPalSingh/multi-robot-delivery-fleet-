# The only thing the brain ever sends downstream to a rover.
#
# This maps 1:1 onto the 8-byte ESP32 downlink frame documented in
# docs/HARDWARE.md, so the wire format needs no translation layer beyond
# packing. The rover holds no planner, no costmap and no goal - it executes
# body velocities and nothing else.

string   rover_id

uint8    MODE_STOP=0
uint8    MODE_VELOCITY=1
uint8    MODE_DOCK=2
uint8    MODE_ESTOP=3
uint8    mode

float64  v_mps                # forward body velocity
float64  w_rps                # yaw rate
uint16   seq                  # wraps at 65535; used for loss and RTT accounting
builtin_interfaces/Time stamp
