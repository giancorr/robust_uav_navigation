# OpenVINS RealSense Dual-VIO EKF Fusion

This repository provides a complete ROS2 (Humble) environment for tracking odometry using **two Intel RealSense T265 cameras** running independent instances of [OpenVINS](https://docs.openvins.com/), and fusing them into a single, perfectly smoothed trajectory using an Extended Kalman Filter (`robot_localization`).

## Architecture

1. **Dual OpenVINS Instances**: 
   - `cam0` (Front): Provides the absolute global pose (XYZ + Roll, Pitch, Yaw) acting as the spatial anchor.
   - `cam1` (Back): Provides redundant absolute position (XYZ) to improve stability and tracking robustness.
2. **Translation Cancellation Node (`odom_to_baselink.cpp`)**: 
   - Receives raw `nav_msgs/Odometry` from both VIOs.
   - Cancels the initial arbitrary translation offsets (the physical 22cm gap between the cameras) and the starting yaw, forcing both sensors to mathematically start exactly at `(0,0,0)` in the global frame.
   - Inflates the VIO measurement covariances by 100x to prevent the EKF from diverging due to sub-millimeter disagreements.
3. **EKF Fusion (`ekf.yaml`)**:
   - Uses `robot_localization` with a heavily tuned low-pass kinematic profile (`process_noise_covariance: 0.0001`).
   - Smoothly averages the two independent trajectories into a single, perfectly stable `base_link` output, completely eliminating high-frequency timestamp jitter and physical flexing vibrations.
4. **Dockerized Environment**:
   - Everything runs inside an isolated Docker container with GUI forwarding for `RViz2` and `PlotJuggler`.

## Running the System

1. Launch the Docker container:
   ```bash
   ./run.sh
   ```
2. Build the workspace (inside Docker):
   ```bash
   colcon build --packages-select odometry_tracker openvins_bringup
   ```
3. Run a TMUX Session:
   ```bash
   cd ~/ros2_ws
   tmuxp load src/pkg/babyk_drone_manager/utils/exploration_optitrack.yml
   ```
   (You can replace `exploration_optitrack.yml` with any other available session config depending on your needs, e.g., `flight_session.yml`, `session.yml`, etc.)
   
## Available TMUX Profiles

Inside the `src/pkg/babyk_drone_manager/utils/` folder, you will find several `.yml` configurations. They are designed for `tmuxp` and allow you to quickly spawn the exact stack you need. They can be grouped by purpose:

### 1. Basic VIO & Estimation
These profiles focus *purely* on camera tracking and state estimation (no autonomous flight components).
- **`session.yml`**: Runs a single OpenVINS instance (Front camera) without EKF. Good for basic debugging.
- **`single_ekf_session.yml`**: Single OpenVINS instance smoothed by the Extended Kalman Filter.
- **`dual_session.yml`**: Runs both Front and Back OpenVINS instances, fused together by the EKF.

### 2. Basic Flight Setup
These profiles launch VIO alongside the MAVROS/MicroXRCE DDS bridge for basic manual/stabilized flight.
- **`flight_session.yml`**: Standard real-world flight (Dual VIO + PX4 Bridge).
- **`flight_session_back.yml`**: Flight relying exclusively on the back camera.
- **`flight_session_optitrack.yml`**: Injects OptiTrack motion capture data instead of VIO. Perfect for ground-truth testing.
- **`flight_session_stereo_ov.yml`**: Uses OpenVINS in a stereo configuration (if applicable) instead of mono.

### 3. Full Autonomous Exploration & Recovery
These profiles launch the complete autonomous stack: Path Planner, Trajectory Interpolator, RTABMap, and the **VIO Recovery FSM**.
- **`exploration.yml`**: The standard full stack for real-world autonomous exploration.
- **`exploration_optitrack.yml`**: Full autonomous stack using OptiTrack for localization. Excellent for testing the VIO Recovery FSM safely without risking a crash due to actual VIO loss.
- **`exploration_back.yml` / `exploration_stereo.yml`**: Exploration using specific camera configurations.
- **`hardware_exploration.yml`**: Full hardware-in-the-loop autonomous exploration.
- **`sewer_exploration.yml` / `warehouse_exploration.yml`**: Exploration profiles with parameters explicitly tuned for narrow/specific environments.

### 4. System Manager & Simulation
These profiles leverage the `babyk_drone_manager` to orchestrate high-level behaviors and simulate environments.
- **`simulation.yml`**: Full PX4 SITL simulation (Gazebo). Launches virtual cameras, RTABMap, and the `autonomous_test_node` to randomly send goals and test the entire stack.
- **`flight.yml`**: The real-world counterpart for the system manager. Launches the full navigation stack and handles centralized commands (takeoff, land, flyto).
- **`test_open_box.yml`**: A utility profile strictly used to test the PX4 actuator commands for the marker-dropping servo.

## Frame Definitions
- `global`: The absolute origin `(0,0,0)` where the system initializes.
- `global_ned`: The North-East-Down orientation of the global frame.
- `base_link`: The exact center of the drone, tracking in FRD (Forward-Right-Down) convention.

## Hardware Setup
- Front Camera (`cam0`): +11 cm along the X-axis of `base_link`.
- Back Camera (`cam1`): -11 cm along the X-axis of `base_link`, physically mounted backwards.
