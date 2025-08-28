# SITL Utilities - PX4 Simulation Environment with ROS2

This repository contains tools and configurations for PX4 SITL (Software In The Loop) simulation integrated with ROS2 Humble in a Docker environment.

## Architecture Overview

The system consists of several modular ROS2 packages, each with a specific responsibility:

```
sitl_utils/
├── ros2_ws-src/           # ROS2 workspace with modular packages
│   ├── drone_odometry2/   # Vehicle odometry publisher (submodule)
│   ├── path_planner/      # 3D trajectory planning (submodule)
│   ├── teleop_node/       # Teleoperation control (submodule)
│   ├── babyk_drone_manager/ # Drone state management and safety (submodule)
│   └── traj_interp/       # Trajectory interpolation with PX4 (submodule)
├── docker/               # Docker configurations
├── models/               # Custom Gazebo models
├── worlds/               # Gazebo worlds for simulation
└── PX4-Autopilot/        # PX4 firmware 
└── PX4_neabotics_/        # PX4 custom firmware 
```

## ROS2 Packages

### 🛸 traj_interp
**Trajectory interpolator with complete PX4 integration**

Implements the ffilter algorithm for smooth trajectory interpolation with integrated PX4 offboard control.

**Key Features:**
- Smooth trajectory interpolation with jerk/acceleration limiting
- Complete PX4 integration: arming/disarming, offboard mode
- Automatic heading calculation based on movement direction
- Smart arming: only on first path or after landing
- Auto-disarming on land detection
- Automatic PX4 mode management

**Topics:**
- **Subscriber:** `/path` (nav_msgs/Path) - Trajectory to follow
- **Publisher:** `/px4_trajectory` (trajectory_msgs/MultiDOFJointTrajectory) - Interpolated trajectory
- **Publisher:** `/fcu/in/vehicle_command` - PX4 commands (arm/disarm)
- **Publisher:** `/fcu/in/offboard_control_mode` - Offboard control mode
- **Subscriber:** `/fcu/out/vehicle_control_mode` - Vehicle mode status
- **Subscriber:** `/fcu/out/vehicle_land_detected` - Landing status

### 📡 drone_odometry2
**Vehicle odometry publisher**

Converts PX4 status messages to standard ROS2 odometry.

**Topics:**
- **Subscriber:** `/fcu/out/vehicle_odometry` (px4_msgs/VehicleOdometry)
- **Publisher:** `/odom` (nav_msgs/Odometry)

### 🗺️ path_planner  
**3D trajectory planner**

Generates optimized 3D paths for drones with obstacle avoidance.

**Topics:**
- **Subscriber:** `/goal_pose` (geometry_msgs/PoseStamped) - Target goal
- **Publisher:** `/path` (nav_msgs/Path) - Planned trajectory

### 🎮 teleop_node
**Teleoperation control**

Interface for manual drone control via keyboard/joystick.

**Topics:**
- **Subscriber:** `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- **Publisher:** `/goal_pose` (geometry_msgs/PoseStamped) - Target pose

### 🛡️ babyk_drone_manager
**State management and safety**

Monitors drone status and implements safety functions.

**Topics:**
- **Subscriber:** `/fcu/out/vehicle_status` (px4_msgs/VehicleStatus)
- **Publisher:** `/safety_status` (std_msgs/Bool) - Safety status

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
git clone --recursive https://github.com/Prisma-Drone-Team/sitl_utils.git
cd sitl_utils
```

### 2. Clone PX4 Firmware
```bash
git clone --single-branch -b release/1.14 git@github.com:PX4/PX4-Autopilot.git --recursive
```

### 3. Clone PX4 Neabotics
```bash
git clone --single-branch -b feature/diffgains_fix_servo_k https://github.com/Neabotics/PX4_neabotics.git --recursive
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

### 6. Initialize Submodules (if needed)
```bash
git submodule update --init --recursive
```

## Development Configuration

### ROS2 Workspace Build
```bash
cd ros2_ws
source install/setup.bash
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

## Usage in simulation with TMUX
```bash
cd ros2_ws
tmuxp load src/babyk_drone_manager/simulation.yml
```

## Trajectory Interpolation Algorithm

The `traj_interp` package implements the second order filter algorithm for smooth path interpolation:

### Technical Features:
- **Jerk Limiting**: Third derivative control for smooth movements
- **Acceleration Limiting**: Respects vehicle dynamic limits  
- **Velocity Limiting**: Maximum speed control
- **Time Integration**: Predictive calculation for real-time control

### Configurable Parameters:
```cpp
// Dynamic limits (m/s, m/s², m/s³)
double max_velocity = 2.0;
double max_acceleration = 1.0; 
double max_jerk = 0.5;

// Time control
double dt = 0.02;  // 50Hz update rate
```

## Important File Structure

```
├── .gitignore          # Excludes build, PX4, core dumps
├── .gitmodules         # Submodule configuration
├── bridge.yaml         # ROS1-ROS2 bridge configuration  
├── docker/
│   ├── Dockerfile
│   └── docker-compose.yml
└── ros2_ws-src/
    └── pkg/
        ├── traj_interp/
        │   ├── include/traj_interp/
        │   │   └── trajectory_interpolator.hpp
        │   ├── src/trajectory_interpolator.cpp
        │   ├── package.xml
        │   └── CMakeLists.txt
        └── [other packages...]
```

## Package Documentation

Each ROS2 package used in this system is documented in its own specific README:

- **traj_interp**: Detailed documentation of the interpolation algorithm and PX4 integration
- **drone_odometry2**: Odometry message conversion specifications
- **path_planner**: 3D planning and obstacle avoidance algorithms
- **teleop_node**: Manual control configuration and interfaces
- **babyk_drone_manager**: Safety system and state monitoring

Refer to the README.md file in each package folder for technical details.

## Important Notes

**PX4 Firmware**: The PX4-Autopilot and PX4_neabotics firmwares must be downloaded separately and are used exclusively for SITL simulation. They are not required for deployment on real hardware.

**PX4_neabotics**: This firmware is specialized for tiltrotor drones and optimized for the Leonardo Drone Contest field, with specific improvements for tiltrotor flight dynamics.
