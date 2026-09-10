cmake_minimum_required(VERSION 3.10)
project(hospital_robot_description)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ── Centralised fleet interfaces ──
# The brain talks to rovers in exactly two messages, RoverCommand down and
# RoverTelemetry up. Work arrives as FleetTask and every decision is published
# in full as AssignmentDecision.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FleetTask.msg"
  "msg/RoverTelemetry.msg"
  "msg/RoverCommand.msg"
  "msg/AssignmentDecision.msg"
  DEPENDENCIES builtin_interfaces
)

# ── Install URDF, meshes, worlds, maps ──
install(DIRECTORY urdf meshes worlds maps
  DESTINATION share/${PROJECT_NAME}
)

# ── Install launch and config ──
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ── Install ESP32 firmware and hardware docs alongside the package ──
install(DIRECTORY firmware docs
  DESTINATION share/${PROJECT_NAME}
)

# ── Install executable scripts ──
install(PROGRAMS
  scripts/fleet_brain.py
  scripts/rover_link.py
  scripts/task_console.py
  scripts/multi_robot_tf_bridge.py
  scripts/cmd_vel_relay.py
  scripts/recovery_helper.py
  scripts/pd_teleop.py
  scripts/odom_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
