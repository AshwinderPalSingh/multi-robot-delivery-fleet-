# Full audit record of one assignment, published on every decision.
#
# Every factor is emitted per candidate, not just the winning total, so the
# choice can be replayed and defended after the fact - including the cases
# where the brain deliberately passes over the nearest rover.
#
# All factor values are in EFFECTIVE SECONDS: F1/F2 are real predicted time,
# F3..F7 are penalties scaled into the same unit so the weighted sum stays
# physically interpretable.

string    task_id
string    winner_id           # "" if the task could not be assigned this cycle
string    reason              # human-readable summary of the decision

string[]  candidate_ids
float64[] score_total
float64[] f1_path_time        # true planner path length / cruise speed
float64[] f2_turn_time        # heading alignment + cumulative yaw along path
float64[] f3_contention       # forecast corridor conflict with committed paths
float64[] f4_energy           # energy margin after mission + return to dock
float64[] f5_reliability      # -ln(P(success)) from the Beta posterior
float64[] f6_localization     # AMCL pose uncertainty
float64[] f7_coverage         # fleet readiness lost by committing this rover
string[]  veto_reason         # empty string when the candidate was eligible

float64   decision_ms
builtin_interfaces/Time stamp
