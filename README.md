# UAV VIO RECOVERY STACK

This repository contains tools and configurations for PX4 SITL (Software In The Loop) simulation, with a specific focus on robust recovery from Visual-Inertial Odometry (VIO) failures using tactile odometry and state machine logic.

## Architecture Overview

The system consists of three main ROS2 packages:

```
uav_motion_stack/
├── ros2_ws-src/                    # ROS2 workspace
│   ├── babyk_drone_manager/        # Drone state management, TF and safety (submodule)
│   ├── open_vins/                  # Visual-Inertial Odometry estimator
│   └── vio_recovery/               # Core recovery logic, tactile odometry, and FSM
├── docker/               # Docker configurations
├── models/               # Custom Gazebo models
├── worlds/               # Gazebo worlds for simulation
├── PX4-Autopilot/        # PX4 firmware 
└── PX4_neabotics/        # PX4 custom firmware 
```

## System Requirements

- **Docker**: For isolated development environment
- **ROS2 Humble**: Robotics framework
- **PX4 v1.14+**: Autopilot firmware
- **Gazebo Garden**: 3D simulator
- **Eigen3**: Mathematical library for matrix operations

## Installation and Setup

A step by step series of examples that tell you how to get a development environment running:

### 1. Repository Clone
```bash
git clone --recursive https://github.com/giancorr/Front-rear-configuration.git -b simulation
cd Front-rear-configuration
```

### 2. Clone PX4 Firmware
```bash
git clone --single-branch -b release/1.14 git@github.com:PX4/PX4-Autopilot.git --recursive
```

### 3. Clone PX4 Neabotics
```bash
git clone --single-branch -b feature/diffgains_fix_servo_k https://github.com/Prisma-Drone-Team/Px4_hcore_autopilot.git PX4_neabotics --recursive
```

### 4. Build Docker Image
```bash
cd docker
docker build -t leo-img -f px4_humble_dockerfile.txt .
```

### 5. Run Container
```bash
./run_cnt.sh
```

### 6. Initialize Submodules
```bash
git submodule update --init --recursive
```

## Development Configuration

### ROS2 Workspace Build
```bash
cd ros2_ws
source install/setup.bash
colcon build --packages-select babyk_drone_manager open_vins vio_recovery --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Usage in simulation with TMUX
```bash
cd ros2_ws
tmuxp load src/pkg/babyk_drone_manager/utils/fr_simulation.yml
```

## Package Documentation

Each ROS2 package used in this system provides specific functionality for the VIO recovery pipeline:

- **babyk_drone_manager**: Handles TF broadcasting, basic movement management, and general safety bounds.
- **open_vins**: The MSCKF-based Visual-Inertial Odometry estimator used as the primary source of pose estimation.
- **vio_recovery**: The core novel package containing the recovery Finite State Machine, tactile odometry, and hardware fallback logic.

---

## ROS2 Packages

### 🚑 vio_recovery
**VIO Failure Recovery & Tactile Odometry System**

Implements advanced fallback mechanisms when the primary VIO (OpenVINS) becomes unstable or degenerates due to lack of visual features (e.g., when facing a blank wall during a spraying mission).

**Key Features:**
- **VIO Recovery FSM (`vio_recovery_fsm`)**: Finite State Machine handling Hover, Strafe, Swipe and Drop maneuvers when VIO fails.
- **Tactile Odometry**: Provides fallback geometric odometry based on physical contact constraints (unilateral projection) when visual tracking is lost.
- **External Wrench Estimator**: Calculates external forces and torques based on drone dynamics, used to detect wall contact.
- **Degeneracy Monitor**: Monitors OpenVINS eigenvalues to preemptively detect tracking degradation.
- **Spray Target Heuristic**: Dynamically selects targets for the spray mission based on visual feature count balance.
- **Surface & Aruco Detectors**: Vision nodes to assist with relocalization and target finding.

### 👓 open_vins
**Visual Inertial Odometry Estimation**

A state-of-the-art filter-based VIO system (Multi-State Constraint Kalman Filter). In this stack, it is specifically configured to output degeneracy metrics (eigenvalues) that are consumed by the `vio_recovery` package.

### 🛡️ babyk_drone_manager
**State management and safety**

Monitors overall drone status and implements safety functions. Also contains the core TMUX simulation files (`fr_simulation.yml`) and TF publishers required to link the simulated Gazebo drone with the ROS2 TF tree and PX4 offboard control.
