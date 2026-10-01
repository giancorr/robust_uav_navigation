# Robust UAV Navigation

Complete ROS 2 (Humble) autonomous flight stack for the **BabyK drone**, integrating real-time Visual-Inertial Odometry (OpenVINS), OptiTrack motion capture, PX4 autopilot, and a VIO Recovery FSM for robust navigation in challenging environments.

## Architecture Overview

```
uav_motion_stack/
├── src/                        # ROS 2 workspace sources
│   ├── pkg/
│   │   ├── babyk_drone_manager/     # High-level flight orchestration + TMUX sessions
│   │   ├── drone_odometry/          # PX4 ↔ ROS 2 frame conversion (NED ↔ ENU)
│   │   ├── path_planner/            # 3D path planning (OMPL + FCL + OctoMap)
│   │   ├── traj_interp/             # Trajectory interpolation + teleop integration
│   │   ├── vio_recovery/            # VIO health monitoring + tactile recovery FSM
│   │   ├── vio_mapping/             # RTABMap-based 3D volumetric mapping
│   │   ├── open_vins/               # OpenVINS visual-inertial odometry
│   │   ├── optitrack_listener/      # OptiTrack NatNet SDK driver (mocap2_driver)
│   │   ├── servo_offboard_control/  # Offboard servo control (marker dropper)
│   │   └── odometry_tracker/        # EKF odometry fusion utilities
│   └── px4_ros_com/                 # PX4 ↔ ROS 2 bridge (uXRCE-DDS)
├── PX4_neabotics/              # PX4 firmware fork (custom vehicle models)
├── models/                     # Gazebo custom models
└── ros2_ws-src/                # Simulation-only workspace overlay
    └── pkg/
        ├── babyk_drone_manager/ # Simulation configs and flight logs
        └── vio_recovery/        # Simulation flight logs
```

## Quick Start

### 1. Build the workspace

```bash
cd ~/ros2_ws
colcon build --packages-select \
  babyk_drone_manager drone_odometry path_planner traj_interp \
  vio_recovery vio_mapping optitrack_listener2
source install/setup.bash
```

### 2. Real Flight (GCS machine)

```bash
# On the GCS laptop — launches RViz, OptiTrack driver, status monitors, flight data logger
tmuxp load src/pkg/babyk_drone_manager/utils/gcs.yml
```

### 3. Real Flight (drone onboard computer)

```bash
cd ~/ros2_ws
# ⭐ Recommended — OptiTrack as localization, pairs with gcs.yml on the GCS machine:
tmuxp load src/pkg/babyk_drone_manager/utils/exploration_optitrack.yml

# Alternative: full OpenVINS-based autonomous exploration:
tmuxp load src/pkg/babyk_drone_manager/utils/hardware_exploration.yml
```

## Available TMUX Profiles

All session files are in `src/pkg/babyk_drone_manager/utils/`.

### Real Hardware

| Session | Description |
|---|---|
| `gcs.yml` | GCS: RViz + OptiTrack driver + topic monitors + flight data logger |
| ⭐ `exploration_optitrack.yml` | **Recommended** onboard session: full autonomous stack using OptiTrack for localization — pair with `gcs.yml` on the GCS machine |
| `hardware_exploration.yml` | Onboard stack with OpenVINS as primary localization (no OptiTrack dependency) |
| `flight_session.yml` | Standard flight with dual VIO (front + back camera) |
| `flight_session_optitrack.yml` | Uses OptiTrack instead of VIO for localization |
| `flight_session_back.yml` | Flight relying exclusively on the back camera |
| `flight_session_stereo_ov.yml` | OpenVINS in stereo configuration |
| `flight.yml` | Full real-flight session (navigation + centralized commands) |
| `test_open_box.yml` | Tests PX4 actuator commands for the marker-dropping servo |

### Estimation Only

| Session | Description |
|---|---|
| `session.yml` | Single OpenVINS instance (front camera), no EKF |
| `single_ekf_session.yml` | Single OpenVINS + EKF smoothing |
| `dual_session.yml` | Front + back OpenVINS fused by EKF |


## Package Summary

| Package | Role |
|---|---|
| `babyk_drone_manager` | High-level command dispatch, move manager, flight data logger |
| `drone_odometry` | NED→ENU conversion, TF publishing, PX4 bridge interface |
| `path_planner` | OMPL/FCL/OctoMap-based 3D path planning |
| `traj_interp` | Trajectory interpolation + teleop integration |
| `vio_recovery` | VIO health monitoring, degeneracy detection, tactile recovery FSM |
| `vio_mapping` | RTABMap 3D volumetric mapping |
| `open_vins` | OpenVINS visual-inertial odometry |
| `optitrack_listener` | OptiTrack NatNet SDK ROS 2 driver (`mocap2_driver`) |
| `servo_offboard_control` | Servo control for the marker-dropping actuator |

## OptiTrack Configuration

The `gcs.yml` session launches `mocap2_driver` automatically. To configure IPs and rigid body IDs, edit:

```
src/pkg/optitrack_listener/launch/mocap2_driver.launch.py
```

Key parameters:
```python
'server_address': "192.168.1.2",   # IP of the PC running Motive
'local_address':  "192.168.1.14",  # IP of the GCS machine
'rigid_body_ids': [3],             # BabyK rigid body ID in Motive
```

See [`optitrack_listener/README.md`](src/pkg/optitrack_listener/README.md) for full parameter reference.

## Frame Conventions

| Frame | Description |
|---|---|
| `map` | Global origin, fixed in world |
| `odom` | Local odometry origin (drifts over time) |
| `base_link` | Drone body center, FRD convention |
| `optitrack_odom` | OptiTrack world frame |

PX4 uses **NED** internally; all ROS 2 nodes operate in **ENU**. The `drone_odometry` package handles the conversion transparently.

## Related READMEs

- [`src/pkg/babyk_drone_manager/README.md`](src/pkg/babyk_drone_manager/README.md) — Flight orchestration, commands, and TMUX sessions
- [`src/pkg/drone_odometry/README.md`](src/pkg/drone_odometry/README.md) — NED↔ENU bridge and TF publishing
- [`src/pkg/path_planner/README.md`](src/pkg/path_planner/README.md) — OMPL path planning and collision avoidance
- [`src/pkg/traj_interp/README.md`](src/pkg/traj_interp/README.md) — Trajectory interpolation modes
- [`src/pkg/vio_recovery/README.md`](src/pkg/vio_recovery/README.md) — VIO health monitoring and recovery FSM
- [`src/pkg/optitrack_listener/README.md`](src/pkg/optitrack_listener/README.md) — OptiTrack NatNet driver
