# A delivery request submitted to the central brain.
#
# Rovers never see this message. It goes to the brain, which decides who
# executes it. On hardware the submitting client is a ward tablet or the
# nurses'-station console talking to the Raspberry Pi over Wi-Fi.

string   task_id
string   station              # named station from fleet_brain.yaml, or "" for a raw pose
float64  goal_x
float64  goal_y
float64  goal_yaw

uint8    PRIORITY_ROUTINE=0
uint8    PRIORITY_NORMAL=1
uint8    PRIORITY_URGENT=2
uint8    PRIORITY_EMERGENCY=3
uint8    priority

float64  payload_kg          # affects the energy model in factor F4
builtin_interfaces/Time stamp
