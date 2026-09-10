# Multi-Robot Delivery Fleet

A centralized multi-robot fleet simulation for hospital logistics, built with ROS 2 Humble, Nav2, and Gazebo Classic. This project demonstrates the orchestration of multiple autonomous rovers operating in a shared hospital environment.

[Insert GIF of the fleet run here]

## Features

- **Multi-Robot Simulation:** 4 independent differential-drive rovers operating simultaneously in Gazebo.
- **Centralized Fleet Brain:** A centralized dispatcher that assigns tasks to the rovers based on a 7-factor weighting policy (distance, battery, capability, etc.).
- **Hospital Environment:** A custom 33x27m simulated hospital ward with 9 distinct delivery stations (nurses station, operating theatre, pharmacy, etc.).
- **Nav2 Stack Integration:** Full autonomous navigation using AMCL for localization, SmacPlanner, and MPPI Controller.
- **Tuned Physics & Localization:** Optimized URDF caster dynamics and AMCL motion models for stable, drift-free localization across the fleet.

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- ROS 2 Navigation2 (Nav2) stack

## Installation

1. Clone the repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/AshwinderPalSingh/multi-robot-delivery-fleet-.git hospital_robot_description
   ```

2. Install dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the package:
   ```bash
   colcon build --packages-select hospital_robot_description
   ```

## Usage

### 1. Launch the Simulation and Fleet

Open a terminal and run the full launch file. This will start Gazebo, RViz, spawn the 4 rovers, initialize the Nav2 stacks, and start the centralized fleet brain.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch hospital_robot_description fleet_full.launch.py
```

Wait approximately 60 seconds for all components (including RViz and the Nav2 stacks) to fully initialize.

### 2. Dispatch a Delivery Task

Open a **new** terminal and publish a task to the fleet dispatcher. The fleet brain will automatically evaluate which rover is best suited for the task and assign it.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub --once /fleet/task hospital_robot_description/msg/FleetTask "{station: 'nurses_station', priority: 1, payload_kg: 2.0}"
```

### Available Stations

You can assign tasks to the following stations defined in the map:
- `nurses_station`
- `operating_theatre`
- `pharmacy`
- `ward_a_bed1`
- `ward_a_bed2`
- `ward_a_bed3`
- `ward_b_bed1`
- `ward_b_bed2`
- `supply_room`

## Author

**Ashwinder Pal Singh**
