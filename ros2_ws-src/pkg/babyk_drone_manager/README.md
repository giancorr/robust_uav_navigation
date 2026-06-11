# Drone Manager

**Centralized management system for drone control and component coordination.**

## Overview

The `babyk_drone_manager` is the main package that manages the entire drone system, coordinating all components and providing a unified interface for control. It centralizes launch files, configurations, and TMUX files for simplified system management.

## Architecture

```
babyk_drone_manager/
├── src/
│   ├── move_manager_node.cpp          # Movement management node
│   └── autonomous_test_node.cpp       # Autonomous testing node
├── include/babyk_drone_manager/
│   ├── move_manager_node.h            # Move manager header
│   └── autonomous_test_node.h         # Autonomous test node header
├── launch/                            # Centralized launch files
│   ├── move_manager.launch.py         # Movement management
│   ├── autonomous_test_node.launch.py # Autonomous testing
│   ├── full_system.launch.py          # Complete system
│   ├── rtabmap_sim.launch.py          # SLAM simulation
│   ├── tf_static_sim.launch.py        # Static TF simulation
│   ├── tf_static_flight.launch.py     # Static TF real flight
│   └── px4_tf_pub_simulation.launch.py # PX4 TF simulation
├── config/                            # Centralized configurations
│   ├── move_manager_params.yaml       # Real flight parameters
│   ├── move_manager_simulation.yaml   # Simulation parameters
│   ├── autonomous_test_node_params.yaml # Test node simulation config
│   └── autonomous_test_node_flight.yaml # Test node real flight config
├── rviz/
│   └── leo.rviz                       # RViz configuration
├── simulation.yml                     # TMUX simulation (with autonomous testing)
└── flight.yml                        # TMUX real flight (with autonomous testing)
```

## Main Components

### Move Manager Node
**Node**: `move_manager_node`  
**Description**: Coordinates drone movements and manages command interface.

**Main Topics**:
- `/move_manager/command` (input) - Movement commands
- `/move_manager/status` (output) - System status
- `/move_base_simple/goal` (output) - Goals for path planner
- `/trajectory_path` (output) - Trajectories for interpolator

### Autonomous Test Node
**Node**: `autonomous_test_node`  
**Description**: Automated testing system that continuously sends random commands to test the entire drone system.

**Features**:
- Monitors `/trajectory_interpolator/status` for system state
- Automatically sends random commands when system is idle
- Intelligent command sequencing (takeoff → flyto/land → repeat)
- Configurable timing and command probabilities

**Main Topics**:
- `/move_manager/command` (output) - Random test commands
- `/trajectory_interpolator/status` (input) - System status monitoring

**Command Sequence**:
1. **Initial**: Sends `takeoff` when system starts
2. **Random Loop**: 90% `flyto(goal1-7)`, 10% `land`  
3. **After Land**: Always sends `takeoff` next

**Configuration** (`config/autonomous_test_node_params.yaml`):
```yaml
autonomous_test_node:
  ros__parameters:
    command_interval_min: 120      # Min seconds between commands
    command_interval_max: 180      # Max seconds between commands  
    max_wait_time: 60             # Max wait if system stuck
    land_probability: 0.10        # 10% chance of land command
```

## Supported Commands

```bash
# Takeoff (maintains current yaw, single waypoint)
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'takeoff'}" --once

# Direct movement (without path planning)
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'go(x,y,z)'}" --once

# Movement with path planning
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'flyto(frame_name)'}" --once

# Landing (configurable depth, maintains current yaw, single waypoint)
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'land'}" --once
# Uses landing_altitude parameter (default: -0.5m = underground landing)

# Emergency stop
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'stop'}" --once

# Command a direct tilting pitch setpoint (TODO create smooth traj for pitch-tilting)
ros2 topic pub --once /fmu/in/tilting_mc_desired_angles px4_msgs/msg/TiltingMcDesiredAngles "{timestamp: $(($(date +%s%N)/1000)), roll_body: 0.1, pitch_body: -0.1}" --once

# Teleop control (activates joystick control)
ros2 topic pub /move_manager/command std_msgs/msg/String "{data: 'teleop'}" --once
```

### Teleop Integration

The move manager includes automatic joystick detection and teleop coordination:

**Joystick Detection**:
- Automatically detects when a joystick is connected to `/joy` topic
- Publishes joystick status on `/move_manager/joystick_connected` (Bool)
- When joystick detected, teleop mode can be activated

**Teleop Command**:
- `teleop` command activates joystick control mode
- Stops any current path execution
- Publishes teleop activation flag on `/move_manager/teleop_active` (Bool)
- Seamlessly integrates with the enhanced_teleop node and trajectory interpolator

**Teleop Integration Flow**:
1. Move manager detects joystick presence
2. User sends `teleop` command to activate teleop mode
3. Move manager publishes `teleop_active: true` flag
4. Enhanced_teleop node responds to flag and begins velocity control
5. Trajectory interpolator integrates velocity increments for smooth control

**Related Topics**:
- `/joy` (input) - Joystick messages for detection
- `/move_manager/joystick_connected` (output) - Joystick status
- `/move_manager/teleop_active` (output) - Teleop mode activation flag
- `/teleop/velocity_increments` (input) - Velocity commands from teleop

## Launch Files

### Complete System
```bash
ros2 launch babyk_drone_manager full_system.launch.py
```
Launches all components: move_manager + path_planner.

### Move Manager
```bash
ros2 launch babyk_drone_manager move_manager.launch.py config_file:=config/move_manager_params.yaml simulation:=false
```

### Autonomous Test Node
```bash
# For simulation
ros2 launch babyk_drone_manager autonomous_test_node.launch.py simulation:=true

# For real flight (conservative timing)
ros2 launch babyk_drone_manager autonomous_test_node.launch.py simulation:=false config_file:=config/autonomous_test_node_flight.yaml
```
Launches the autonomous testing system that continuously sends random commands.

### RTABMap Simulation
```bash
ros2 launch babyk_drone_manager rtabmap_sim.launch.py use_sim_time:=true
```

### TF Static Publishers
```bash
# For simulation (large arena with wide-spaced goals)
ros2 launch babyk_drone_manager tf_static_sim.launch.py use_sim_time:=true

# For real flight (small arena with close-spaced goals)  
ros2 launch babyk_drone_manager tf_static_flight.launch.py use_sim_time:=false
```

## TMUX Configurations

### Complete Simulation
```bash
tmuxp load simulation.yml
```

**System started**:
- PX4 SITL + Gazebo
- MicroXRCE Agent
- Gazebo-ROS Bridge
- RTABMap SLAM
- RViz
- TF Publishers
- Move Manager
- Path Planner
- Trajectory Interpolator
- **Autonomous Test Node** (sends random commands)
- PlotJuggler

**Autonomous Testing**: The system automatically starts sending random commands for continuous testing of all drone functions.

### Real Flight
```bash
tmuxp load flight.yml
```

**System started**:
- Move Manager
- Path Planner  
- Trajectory Interpolator
- SLAM (Leonardo)
- **TF Static Publishers** (goal1-7 for real arena)
- **Autonomous Test Node** (conservative timing for real flight)
- RViz

**Real Arena Configuration**: Optimized for small arena (5x6 meters) with goals positioned safely within bounds.

## 🛡️ Safety Parameters

### Flight Altitude Control
The system now provides precise control over takeoff and landing altitudes:

**Takeoff Configuration**:
- `takeoff_altitude`: Height for takeoff operations (default: 1.5m)
- Single waypoint path to avoid horizontal drift

**Landing Configuration**:
- `landing_altitude`: Target depth for landing (default: -0.5m)
- Negative values = underground landing for complete safety
- Maintains current position (X,Y) during landing

**Safety Benefits**:
- ✅ **Underground landing**: Ensures complete drone shutdown
- ✅ **Configurable depths**: Adapt to different ground conditions
- ✅ **Position stability**: No horizontal movement during critical phases
- ✅ **Failsafe behavior**: Predictable landing regardless of conditions

**Example Configurations**:
```yaml
# Standard underground landing
landing_altitude: -0.5

# Surface landing  
landing_altitude: 0.0

# Deep underground (soft surfaces)
landing_altitude: -1.0
```

## Configuration Parameters

### move_manager_params.yaml (Real Flight)
```yaml
move_manager_node:
  ros__parameters:
    command_topic: "/move_manager/command"
    status_topic: "/move_manager/status"
    takeoff_altitude: 1.5     # Takeoff height (meters)
    landing_altitude: -0.5    # Landing depth (meters, negative = underground)
    simulation: false
```

### move_manager_simulation.yaml (Simulation)
```yaml
move_manager_node:
  ros__parameters:
    command_topic: "/move_manager/command"
    status_topic: "/move_manager/status"
    takeoff_altitude: 1.5     # Takeoff height (meters)
    landing_altitude: -0.5    # Landing depth (meters, negative = underground) 
    simulation: true  # Enables TF publishing
```

## Autonomous Testing

### Overview
The `autonomous_test_node` provides continuous automated testing of the entire drone system by sending random commands. This ensures comprehensive testing of all flight modes and system components.

### Usage

**Start Complete Autonomous Testing**:
```bash
tmuxp load simulation.yml
```
The autonomous test node is automatically included and starts testing immediately.

**Manual Launch**:
```bash
ros2 launch babyk_drone_manager autonomous_test_node.launch.py simulation:=true
```

**Monitor Testing**:
```bash
# Watch commands being sent
ros2 topic echo /move_manager/command

# Monitor system status
ros2 topic echo /trajectory_interpolator/status

# Check autonomous test node logs
ros2 node info /autonomous_test_node
```

### Configuration

Edit `config/autonomous_test_node_params.yaml` to customize:

```yaml
autonomous_test_node:
  ros__parameters:
    command_interval_min: 120      # Minimum time between commands (seconds)
    command_interval_max: 180      # Maximum time between commands (seconds)
    max_wait_time: 60             # Max time to wait if system is stuck
    land_probability: 0.10        # Probability of sending land command (0.0-1.0)
```

### Test Sequence

1. **System Initialization**: Waits for system to be idle
2. **Initial Takeoff**: Sends `takeoff` command
3. **Random Commands**: 
   - 90% chance: `flyto(goal1)` through `flyto(goal7)`
   - 10% chance: `land`
4. **Smart Recovery**: After land, next command is always `takeoff`
5. **Continuous Loop**: Repeats indefinitely for stress testing

### Benefits

- **Comprehensive Testing**: Tests all flight modes automatically
- **Stress Testing**: Continuous operation reveals system issues
- **Hands-Free**: No manual intervention required
- **Configurable**: Adjustable timing and command probabilities
- **Intelligent**: Responds to system status and recovers from errors

## System Integration

The `babyk_drone_manager` coordinates:

1. **Path Planner** (`path_planner`) - Path planning with OMPL+FCL
2. **Trajectory Interpolator** (`traj_interp`) - Interpolation and trajectory resampling
3. **Drone Odometry** (`drone_odometry2`) - TF publishing and PX4 odometry
4. **RTABMap** - Visual SLAM for environmental mapping

## System States

- `IDLE` - System ready
- `PLANNING_PATH` - Path planning in progress
- `EXECUTING_TRAJECTORY` - Trajectory execution
- `TAKING_OFF` - Takeoff in progress
- `LANDING` - Landing in progress
- `STOPPED` - System stopped
- `ERROR_*` - Various error states

## Troubleshooting

### Common Issues

1. **Move Manager not found**:
   ```bash
   colcon build --packages-select babyk_drone_manager
   source install/setup.bash
   ```

2. **TF not published in simulation**:
   - Verify `simulation: true` in parameters
   - Check that px4_tf_pub_simulation.launch.py is active

3. **Commands not responding**:
   - Verify topic: `ros2 topic echo /move_manager/status`
   - Check odometry: `ros2 topic echo /px4/odometry/out`

4. **Path planning fails**:
   - Verify octomap: `ros2 topic echo /octomap_binary`
   - Check workspace limits in path_planner config

## Dependencies

**ROS 2 Packages**:
- `rclcpp`, `nav_msgs`, `geometry_msgs`, `std_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `rtabmap_ros`, `rtabmap_util`, `sensor_msgs`
- `image_transport`, `visualization_msgs`

**Required Custom Packages** (for complete flight stack):
- `path_planner` - Path planning with OMPL+FCL
- `traj_interp` - Trajectory interpolation and resampling
- `drone_odometry2` - TF publishing and PX4 odometry integration

**Optional Packages**:
- `joy` - Enable usb joystick connection 
- `teleop_node` - Enhanced teleop control for manual flight

## Build and Installation

**Prerequisites**: Ensure all required custom packages are available in your workspace:
```bash
# Check that all packages are present
ls ~/ros2_ws/src/pkg/
# Should contain: babyk_drone_manager, path_planner, traj_interp, drone_odometry2
```

**Build complete flight stack**:
```bash
# Build all required packages
cd ~/ros2_ws
colcon build --packages-select babyk_drone_manager path_planner traj_interp drone_odometry2

# Source workspace
source install/setup.bash

# Verify installation
ros2 launch babyk_drone_manager move_manager.launch.py --help
ros2 launch babyk_drone_manager autonomous_test_node.launch.py --help
```

**Individual package build**:
```bash
# Build only babyk_drone_manager (requires other packages to be built first)
cd ~/ros2_ws
colcon build --packages-select babyk_drone_manager
source install/setup.bash
```

## Development Notes

- **Simulation Mode**: Enables automatic TF publishing for Gazebo integrations
- **Real Mode**: Disables TF publishing, uses real hardware
- **Replan Logic**: Maximum 5 attempts for path planning
- **Emergency Stop**: `stop` command immediately interrupts any movement
