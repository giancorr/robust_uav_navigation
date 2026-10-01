# ROBUST UAV NAVIGATION

This repository contains the full software stack for autonomous UAV navigation and exploration, with a specific focus on robust recovery from Visual-Inertial Odometry (VIO) failures using tactile odometry and finite state machine logic. The stack has been validated both in Gazebo simulation and on real hardware (Intel RealSense T265 + OptiTrack).

## Architecture Overview

```
uav_motion_stack/
├── ros2_ws-src/                    # ROS2 workspace
│   ├── babyk_drone_manager/        # Central mission manager: commands, safety, TF, GCS, logging
│   ├── drone_odometry/             # Odometry processing and PX4 ENU interface
│   ├── open_vins/                  # Visual-Inertial Odometry estimator (personal fork)
│   ├── optitrack_listener/         # OptiTrack/NatNet driver (real flight ground truth)
│   ├── path_planner/               # Global 3D path planning (OMPL + FCL + OctoMap)
│   ├── traj_interp/                # Trajectory interpolation and smooth setpoint generation
│   ├── vio_mapping/                # Probabilistic OctoMap generation from OpenVINS features
│   └── vio_recovery/               # VIO failure recovery FSM and tactile odometry
├── docker/                         # Docker configurations
├── models/                         # Custom Gazebo models
├── worlds/                         # Gazebo worlds for simulation
└── PX4_neabotics/                  # PX4 custom firmware (Neabotics fork, SITL only)
```

## System Requirements

- **Docker**: For isolated development environment
- **ROS2 Humble**: Robotics framework
- **PX4 v1.14+**: Autopilot firmware
- **Gazebo Garden**: 3D simulator (simulation only)
- **Eigen3**: Mathematical library for matrix operations

## Installation and Setup

### 1. Repository Clone
```bash
git clone --recursive https://github.com/giancorr/robust_uav_navigation.git -b simulation
cd robust_uav_navigation
```

### 2. Clone PX4 Neabotics Firmware (simulation only)
```bash
git clone --single-branch -b vio https://github.com/Prisma-Drone-Team/Px4_hcore_autopilot.git PX4_neabotics --recursive
```

### 3. Build Docker Image
```bash
cd docker
docker build -t leo-img -f px4_humble_dockerfile.txt .
```

### 4. Run Container
```bash
./run_cnt.sh
```

### 5. Initialize Submodules
```bash
git submodule update --init --recursive
```

## Development Configuration

### ROS2 Workspace Build
```bash
cd ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Main Dependencies
```xml
<!-- Common package.xml -->
<depend>rclcpp</depend>
<depend>px4_msgs</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<depend>trajectory_msgs</depend>
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>eigen3_cmake_module</depend>
```

## Usage (TMUX sessions)

All scenarios are launched via `tmuxp` from inside the Docker container.

### 🏭 Warehouse Exploration (simulation)

Autonomous exploration of a large warehouse-like environment. The `warehouse_test_node` uses frontier-based goal selection driven by the live OctoMap built by `vio_mapping`, enabling the drone to autonomously navigate without pre-defined waypoints.

```bash
cd ros2_ws
tmuxp load src/pkg/babyk_drone_manager/utils/warehouse_exploration.yml
```

**Config**: `open_vins/config/baby_k_warehouse/` · `vio_recovery/config/params_warehouse.yaml`

### 🏢 Corridor Exploration (simulation)

Fixed-waypoint exploration in a narrow corridor with VIO recovery enabled.

```bash
cd ros2_ws
tmuxp load src/pkg/babyk_drone_manager/utils/exploration.yml
```

**Config**: `open_vins/config/baby_k/` · `vio_recovery/config/params_corridor.yaml`

### 🕳️ Sewer Exploration (simulation)

Exploration in a featureless, dark pipe environment. Uses the most aggressive VIO recovery parameters.

```bash
cd ros2_ws
tmuxp load src/pkg/babyk_drone_manager/utils/sewer_exploration.yml
```

**Config**: `open_vins/config/baby_k_sewer/` · `vio_recovery/config/params_sewer.yaml`

### ✈️ Real Flight (GCS)

Ground Control Station session for real hardware flights. Connects to the drone over the network and monitors all key topics.

```bash
tmuxp load src/pkg/babyk_drone_manager/utils/gcs.yml
```

**Stack launched**: RViz · OptiTrack driver · Flight data logger · Topic monitors

## Package Documentation

- **babyk_drone_manager**: Central mission manager. Handles high-level commands (`takeoff`, `flyto`, `land`), TF broadcasting, safety bounds, GCS TMUX sessions, and flight data logging.
- **drone_odometry**: Provides odometry conversion and the PX4 ENU interface for the flight controller.
- **open_vins**: MSCKF-based VIO estimator. Configured to output degeneracy eigenvalues consumed by `vio_recovery`. Drone-specific configs (T265, dual-camera) are maintained in `config/`.
- **optitrack_listener**: NatNet SDK driver that publishes rigid body poses from Motive as `nav_msgs/Odometry` on `/optitrack/body_<ID>/odometry`. Used as ground truth during real flights.
- **path_planner**: Global 3D collision-free path planning using OMPL (RRT*) and FCL. Reads the OctoMap from `vio_mapping` for obstacle representation.
- **traj_interp**: Interpolates sparse waypoints from the path planner into a high-frequency stream of smooth trajectory setpoints for PX4 Offboard mode. Includes teleop integration.
- **vio_mapping**: Builds a probabilistic OctoMap from OpenVINS point cloud features using a MonoSpheres-inspired pipeline. Supports dual-camera fusion. Used by the path planner and the warehouse explorer.
- **vio_recovery**: The core novel package. Contains the recovery Finite State Machine (`NAVIGATE → STOP → STRAFE → SETTLE → SWIPE → RETURN`), tactile odometry, degeneracy monitor, and external wrench estimator.

## Important Notes

**PX4_neabotics**: This firmware (Neabotics fork, `vio` branch) is specialized for tiltrotor drones and used exclusively for SITL simulation; it is not required for real hardware.

**Real hardware setup**: Real flights use an Intel RealSense T265 for VIO (via OpenVINS) and an OptiTrack motion capture system as ground truth. The `optitrack_listener` package publishes ground truth odometry, which is also fed to PX4 as visual odometry via `scripts/PX4_odom_publisher.py`.

---

## ROS2 Packages

### 🚑 vio_recovery
**VIO Failure Recovery & Tactile Odometry System**

Implements advanced fallback mechanisms when VIO (OpenVINS) becomes unstable or degenerates due to lack of visual features.

**Key Features:**
- **VIO Recovery FSM**: State machine handling Hover, Strafe, Swipe and Drop maneuvers when VIO fails.
- **Tactile Odometry**: Fallback geometric odometry based on physical contact constraints (unilateral projection).
- **External Wrench Estimator**: Calculates external forces/torques to detect wall contact.
- **Degeneracy Monitor**: Monitors OpenVINS eigenvalues to preemptively detect tracking degradation.
- **Target Heuristic**: Selects strafe direction (LEFT/RIGHT) based on visual feature count balance.

### 👓 open_vins
**Visual Inertial Odometry Estimation (Personal Fork)**

A state-of-the-art MSCKF-based VIO system imported as a Git submodule. Configured to output degeneracy metrics consumed by `vio_recovery`. Supports dual fisheye cameras (T265 front + back).

### 🛡️ babyk_drone_manager
**Central Mission Manager**

Handles high-level flight commands, TF broadcasting, safety bounds, and all TMUX session files. Also contains:
- `flight_data_logger`: C++ node that records all flight data (VIO, OptiTrack, PX4, FSM state, eigenvalues, wrenches) to the `flight_logs/` folder.
- `warehouse_test_node`: Frontier-based autonomous exploration node that uses the live OctoMap to generate goals dynamically.
- `exploration_node`: Fixed-waypoint exploration node for corridor and sewer scenarios.

### 📡 optitrack_listener
**OptiTrack / NatNet Ground Truth Driver**

Connects to a Motive server via the NatNet SDK and publishes rigid body poses as `nav_msgs/Odometry` on `/optitrack/body_<ID>/odometry`.

### 📏 drone_odometry
**Odometry Processing and PX4 ENU Interface**

Handles odometry source conversion and TF publishing to make localization data available to the rest of the stack in ENU coordinates.

### 🗺️ path_planner
**Autonomous 3D Path Generation**

Generates collision-free paths using OMPL (RRT*) and FCL. Reads obstacle maps from `vio_mapping` via OctoMap messages.

### 📈 traj_interp
**Trajectory Interpolation**

Converts sparse waypoints into a continuous high-frequency stream of smooth setpoints for PX4 Offboard mode. Supports teleop integration via joystick.

### 🗺️ vio_mapping
**MonoSpheres-style OctoMap Builder**

Builds a probabilistic OctoMap from OpenVINS feature point clouds using a sparse mesh + free space polyhedron pipeline, with native dual-camera support. Publishes `octomap_msgs/Octomap` directly to the path planner.
