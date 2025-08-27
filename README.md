# SITL Utilities - PX4 Simulation Environment with ROS2

This repository contains tools and configurations for PX4 SITL (Software In The Loop) simulation integrated with ROS2 Humble in a Docker environment.

## Architecture Overview

The system consists of several modular ROS2 packages, each with a specific responsibility:

```
sitl_utils/
├── ros2_ws-src/           # ROS2 workspace with modular packages
│   ├── drone_odometry2/   # Vehicle odometry publisher
│   ├── path_planner/      # 3D trajectory planning
│   ├── teleop_node/       # Teleoperation control
│   ├── babyk_drone_manager/ # Drone state management and safety
│   └── traj_interp/       # Trajectory interpolation with PX4
├── docker/               # Docker configurations
├── models/               # Custom Gazebo models
├── worlds/               # Gazebo worlds for simulation
└── PX4-Autopilot/        # PX4 firmware (submodule)
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
cd ros2_ws-src
colcon build --packages-select drone_odometry2 path_planner teleop_node babyk_drone_manager traj_interp
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

## Usage

### 1. Complete System Launch
```bash
# Terminal 1: PX4 SITL
cd PX4-Autopilot
make px4_sitl gazebo-classic

# Terminal 2: ROS2 Packages
ros2 launch sitl_utils complete_system.launch.py
```

### 2. Send Trajectory
```bash
# Publish a sample trajectory
ros2 topic pub /path nav_msgs/Path '{
  header: {frame_id: "map"},
  poses: [
    {pose: {position: {x: 0, y: 0, z: 5}}},
    {pose: {position: {x: 10, y: 0, z: 5}}},
    {pose: {position: {x: 10, y: 10, z: 5}}}
  ]
}'
```

### 3. Monitoring
```bash
# System status
ros2 topic echo /safety_status

# Odometry
ros2 topic echo /odom

# Interpolated trajectory  
ros2 topic echo /px4_trajectory
```

## ffilter Algorithm

The `traj_interp` package implements the ffilter algorithm for smooth interpolation:

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

## Troubleshooting

### Common Issues:

1. **PX4 won't arm**
   - Check PX4 safety parameters
   - Ensure vehicle is in MANUAL or STABILIZED mode

2. **Trajectory not followed**
   - Verify publication on `/path`
   - Check that `traj_interp` is running

3. **Build errors**
   - Verify Eigen3 dependencies: `sudo apt install libeigen3-dev`
   - Check ROS2 Humble version

### Logging and Debug:
```bash
# Detailed ROS2 logs
ros2 run traj_interp trajectory_interpolator --ros-args --log-level DEBUG

# Monitor active topics
ros2 topic list

# Info on specific topic
ros2 topic info /px4_trajectory
```

## Contributing

1. Fork the repository
2. Create feature branch: `git checkout -b feature/new-feature`
3. Commit changes: `git commit -am 'Add new feature'`
4. Push branch: `git push origin feature/new-feature`  
5. Create Pull Request

## License

This project is distributed under the MIT license. See `LICENSE` file for details.

## Support

For support and questions:
- **Issues**: [GitHub Issues](https://github.com/Prisma-Drone-Team/sitl_utils/issues)
- **Documentation**: [Project Wiki](https://github.com/Prisma-Drone-Team/sitl_utils/wiki)
- **Contact**: team@prisma-drone.com
