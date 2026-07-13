# VIO Recovery

A ROS 2 package designed to handle **Visual-Inertial Odometry (VIO) failures** during autonomous drone navigation in confined and challenging environments (e.g., textureless corridors, dark tunnels). 

When the primary visual odometry degrades, this package seamlessly takes over by utilizing a Finite State Machine (FSM) to switch the drone to a blind tactile navigation mode. The drone will stop, strafe to find a physical wall, slide along the wall to stabilize, and leave visual breadcrumbs (ground drops and wall stains) before resuming its mission.

## 🚀 Core Features

*   **VIO Health Monitoring:** Monitors the covariance matrix (eigenvalues) of the VIO output to detect degeneracy or imminent tracking loss.
*   **Tactile Odometry:** A safety layer that takes over when VIO fails. It generates a localized, open-loop kinematic projection to keep the PX4 flight controller stable while flying blind.
*   **External Wrench Estimation:** Analyzes the physical forces and torques acting on the drone to detect wall collisions during the blind strafing phase.
*   **Recovery FSM (Finite State Machine):** Orchestrates the recovery sequence (`NAVIGATE` → `STOP` → `STRAFE` → `SETTLE` → `SWIPE` → `RETURN`).
*   **Visual Breadcrumbs (Simulation):** Provides spawner nodes (`drop_spawner` and `swipe_spawner`) to dynamically generate ground markers and wall stains in Gazebo during the recovery maneuver.

## 🧩 Architecture

The package is composed of the following key nodes:

1.  **`vio_recovery_fsm_node`**: The brain of the recovery system. It subscribes to health status and external wrenches, managing the state transitions to safely interact with the environment.
2.  **`tactile_odometry_node`**: Intercepts the trajectory setpoints and physically projects them based on IMU/velocity integration during VIO failure. It completely spoofs the `/fmu/in/vehicle_visual_odometry` topic for PX4.
3.  **`degeneracy_monitor_node`**: Analyzes the VIO state and publishes a health status (`HEALTHY` or `INCONSISTENT`).
4.  **`external_wrench_estimator_node`**: Calculates physical impacts by observing the discrepancy between commanded thrust and actual drone acceleration.
5.  **`drop_spawner_node` / `swipe_spawner_node`**: Simulation-only nodes that use ROS 2 services to spawn visual markers dynamically inside Gazebo.
6.  **`spray_target_heuristic_node`**: A heuristic vision node to decide the safest direction (LEFT or RIGHT) to strafe when initiating the recovery sequence.

## ⚙️ Configuration

The package behavior can be tuned via the `config/params.yaml` file. 

Key parameters include:
*   `impact_force_threshold`: The force (in Newtons) required to trigger a wall collision detection.
*   `strafe_velocity` / `strafe_timeout`: How fast and how long the drone should search for a wall.
*   `swipe_length` / `swipe_velocity`: The duration and speed of the tactile sliding motion along the wall.
*   `eigenvalue_threshold`: The maximum allowed uncertainty in the VIO covariance before triggering a failure.

## 🛠️ Usage

To launch the entire VIO recovery stack alongside the main simulation, use the provided launch file:

```bash
ros2 launch vio_recovery launch_recovery.launch.py
```
