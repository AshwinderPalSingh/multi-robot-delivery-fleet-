# Everything the central brain knows about one rover.
#
# In simulation rover_link synthesises this from Gazebo topics. On hardware
# the same node builds it from the ESP32 uplink frame (encoder ticks, battery
# ADC, IMU) plus the brain-side AMCL estimate. The brain is written against
# this message only, so the two cases are indistinguishable to it.

string   rover_id

uint8    LINK_OFFLINE=0
uint8    LINK_OK=1
uint8    LINK_STALE=2         # packets late; pose is dead-reckoned and decaying
uint8    link_state

uint8    STATE_IDLE=0
uint8    STATE_EN_ROUTE=1
uint8    STATE_SERVING=2
uint8    STATE_RETURNING=3
uint8    STATE_BLOCKED=4
uint8    STATE_FAULT=5
uint8    STATE_CHARGING=6
uint8    mission_state

float64  x
float64  y
float64  yaw
float64  pose_sigma           # sqrt(trace of AMCL xy covariance), metres - factor F6

float64  v                    # measured body velocities, from wheel encoders
float64  w

float64  battery_wh           # remaining usable energy - factor F4
float64  battery_volts
float64  motor_current_a

uint32   comms_rtt_ms
string   active_task_id
float64  odom_total_m         # lifetime distance, feeds the reliability prior

builtin_interfaces/Time stamp
