#include "vio_recovery/vio_recovery_fsm.hpp"
#include <sstream>

using namespace std::chrono_literals;

VioRecoveryFSM::VioRecoveryFSM()
    : Node("vio_recovery_fsm"),
      current_state_(DroneState::NAVIGATE),
      stop_command_sent_(false),
      drop_command_sent_(false),
      swipe_command_sent_(false),
      return_command_sent_(false),
      impact_detected_(false),
      recovery_start_x_(0.0), recovery_start_y_(0.0), recovery_start_z_(0.0), recovery_start_yaw_(0.0),
      current_x_(0.0), current_y_(0.0), current_z_(0.0), current_yaw_(0.0),
      odometry_received_(false),
      armed_(false),
      consecutive_consistent_count_(0),
      strafe_direction_("RIGHT"),
      direction_received_(false),
      return_start_y_(0.0),
      swipe_vx_body_(0.0), swipe_vy_body_(0.0)
{
    // Parameters
    this->declare_parameter<bool>("enable_recovery",          true);
    this->declare_parameter<bool>("enable_lateral",           true);
    this->declare_parameter<bool>("enable_bottom",            true);
    this->declare_parameter<bool>("trigger_on_potentially_inconsistent", false);
    this->declare_parameter<int>("min_consistent_to_arm",         10);
    this->declare_parameter<double>("delay_before_drop",      1.5);
    this->declare_parameter<double>("strafe_velocity", 0.1);
    this->declare_parameter<double>("return_velocity", 0.1);
    this->declare_parameter<double>("return_distance", 1.0);
    this->declare_parameter<double>("max_strafe_distance", 5.0);    // m
    this->declare_parameter<double>("swipe_length",           1.0);   // m
    this->declare_parameter<double>("swipe_velocity",         0.20);   // m/s
    this->declare_parameter<double>("impact_force_threshold", 0.5);   // N

    enable_recovery_        = this->get_parameter("enable_recovery").as_bool();
    enable_lateral_         = this->get_parameter("enable_lateral").as_bool();
    enable_bottom_          = this->get_parameter("enable_bottom").as_bool();
    trigger_on_potentially_inconsistent_ = this->get_parameter("trigger_on_potentially_inconsistent").as_bool();
    min_consistent_to_arm_     = this->get_parameter("min_consistent_to_arm").as_int();
    delay_before_drop_      = this->get_parameter("delay_before_drop").as_double();
    strafe_velocity_        = this->get_parameter("strafe_velocity").as_double();
    return_velocity_        = this->get_parameter("return_velocity").as_double();
    return_distance_        = this->get_parameter("return_distance").as_double();
    max_strafe_distance_    = this->get_parameter("max_strafe_distance").as_double();
    strafe_timeout_         = max_strafe_distance_ / (std::abs(strafe_velocity_) > 0.01 ? std::abs(strafe_velocity_) : 0.01);
    swipe_velocity_         = this->get_parameter("swipe_velocity").as_double();
    swipe_length_           = this->get_parameter("swipe_length").as_double();
    swipe_duration_         = swipe_length_ / swipe_velocity_;
    impact_force_threshold_ = this->get_parameter("impact_force_threshold").as_double();



    RCLCPP_INFO(this->get_logger(), 
        "VioRecoveryFSM initialized. swipe_length=%.2f, swipe_vel=%.2f -> duration=%.2f s", 
        swipe_length_, swipe_velocity_, swipe_duration_);

    // Subscriptions
    sub_health_ = this->create_subscription<std_msgs::msg::String>(
        "/vio_health_status", 10,
        std::bind(&VioRecoveryFSM::health_callback, this, std::placeholders::_1));

    // Configure SensorData QoS for odometry
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/px4/odometry/out", qos,
        std::bind(&VioRecoveryFSM::odometry_callback, this, std::placeholders::_1));

    sub_wrench_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/drone/external_wrench", 10,
        std::bind(&VioRecoveryFSM::wrench_callback, this, std::placeholders::_1));

    sub_heuristic_ = this->create_subscription<std_msgs::msg::String>(
        "/decision/spray_target", 10,
        std::bind(&VioRecoveryFSM::heuristic_callback, this, std::placeholders::_1));

    sub_interp_cmd_yaw_ = this->create_subscription<std_msgs::msg::Float64>(
        "/trajectory_interpolator/cmd_yaw", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
            interp_cmd_yaw_ = msg->data;
            interp_cmd_yaw_received_ = true;
        });

    // Publishers
    pub_move_manager_cmd_ = this->create_publisher<std_msgs::msg::String>("/move_manager/command", 10);
    pub_drone_cmd_        = this->create_publisher<std_msgs::msg::String>("/seed_pdt_drone/command", 10);
    pub_drop_cmd_         = this->create_publisher<std_msgs::msg::Bool>("/command/drop_marker", 10);
    pub_swipe_cmd_        = this->create_publisher<std_msgs::msg::String>("/command/swipe_paint", 10);
    pub_ff_cmd_vel_       = this->create_publisher<geometry_msgs::msg::Twist>("/recovery/ff_cmd_vel", 10);
    pub_target_yaw_       = this->create_publisher<std_msgs::msg::Float64>("/recovery/target_yaw", 10);
    pub_strafe_direction_ = this->create_publisher<std_msgs::msg::String>("/recovery/strafe_direction", 10);
    state_pub_            = this->create_publisher<std_msgs::msg::String>("/fsm/current_state", 10);
    pub_fsm_state_num_    = this->create_publisher<std_msgs::msg::Int32>("/fsm/current_state_num", 10);
    pub_goal_             = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);
    pub_teleop_active_    = this->create_publisher<std_msgs::msg::Bool>("/move_manager/teleop_active", 10);

    // FSM loop at 10 Hz
    timer_ = this->create_wall_timer(100ms, std::bind(&VioRecoveryFSM::fsm_loop, this));

    // Initialize with the node's clock so time sources match (sim vs system)
    node_start_time_ = this->now();
    state_entry_time_ = this->now();
    last_fsm_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "VIO Recovery FSM Initialized");
}

// ─────────────────────────────────────────────────────────────────────────────
// Callbacks
// ─────────────────────────────────────────────────────────────────────────────

void VioRecoveryFSM::health_callback(const std_msgs::msg::String::SharedPtr msg) {
    // If recovery is disabled by parameter, do nothing and let the drone crash naturally
    if (!enable_recovery_) {
        return;
    }

    // Arm check: only trigger recovery if VIO has first been stable for N consecutive CONSISTENT messages
    if (msg->data == "CONSISTENT" || msg->data == "POTENTIALLY_INCONSISTENT" ||
        msg->data == "POTENTIALLY_CONSISTENT") {
        if (msg->data == "CONSISTENT") {
            consecutive_consistent_count_++;
        } else if (msg->data == "POTENTIALLY_CONSISTENT") {
            // Recovering from INCONSISTENT: count very conservatively (no decrement, no increment)
            // Just hold and wait for full CONSISTENT before arming again
        } else {
            // POTENTIALLY_INCONSISTENT: slightly degrades confidence but doesn't reset it
            consecutive_consistent_count_ = std::max(0, consecutive_consistent_count_ - 1);
        }
        if (!armed_ && consecutive_consistent_count_ >= min_consistent_to_arm_) {
            armed_ = true;
            RCLCPP_INFO(this->get_logger(), "[HEALTH] FSM ARMED after %d consecutive CONSISTENT messages.", consecutive_consistent_count_);
        }
        return;
    }

    // Not armed yet — ignore inconsistency
    if (!armed_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[HEALTH] Ignoring INCONSISTENT: FSM not yet armed (consistent count: %d/%d).",
            consecutive_consistent_count_, min_consistent_to_arm_);
        return;
    }

    bool trigger_condition = false;
    if (msg->data == "INCONSISTENT") {
        // Reset confidence so FSM must re-arm before triggering again
        consecutive_consistent_count_ = 0;
        armed_ = false;
        trigger_condition = true;
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[HEALTH] Unknown health message: '%s'", msg->data.c_str());
    }

    if (current_state_ == DroneState::NAVIGATE && trigger_condition) {

        RCLCPP_WARN(this->get_logger(), "VIO INCONSISTENCY DETECTED! Entering STOP state.");
        current_state_     = DroneState::STOP;
        state_entry_time_  = this->now();
        stop_command_sent_ = false;
        drop_command_sent_ = false;
        impact_detected_   = false;
        bad_impact_        = false;
        swipe_command_sent_ = false;
        return_command_sent_ = false;

        // Save current position and yaw as "center to return to"
        if (odometry_received_) {
            recovery_start_x_ = current_x_;
            recovery_start_y_ = current_y_;
            recovery_start_z_ = current_z_;
            recovery_start_yaw_ = current_yaw_;
        }
    }
}

void VioRecoveryFSM::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    current_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    odometry_received_ = true;
}

void VioRecoveryFSM::wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    double fx = msg->wrench.force.x;
    double fy = msg->wrench.force.y;
    double fz = msg->wrench.force.z;
    double force_magnitude = std::sqrt(fx*fx + fy*fy + fz*fz);

    // Process wrench based on state
    if (current_state_ == DroneState::STRAFE && !impact_detected_) {
        if (bad_impact_) return; // already failed, waiting for FSM loop

        auto now = this->now();
        auto elapsed = (now - state_entry_time_).seconds();
        if (elapsed >= 0.5 && force_magnitude >= impact_force_threshold_) {
            RCLCPP_INFO(this->get_logger(), "[STRAFE] Wall impact detected! Force: %.2f N", force_magnitude);
            impact_detected_ = true;
            
            // Calculate parallel swipe direction from impact normal
            double fx = msg->wrench.force.x;
            double fy = msg->wrench.force.y;
            double theta_f = std::atan2(fy, fx);
            
            // Determine expected force angle based on wall side
            double target_force_angle = (strafe_direction_ == "LEFT") ? -M_PI_2 : M_PI_2;
            
            double delta_yaw = theta_f - target_force_angle;
            while (delta_yaw >  M_PI) delta_yaw -= 2.0 * M_PI;
            while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;
            
            // Safety: Abort if impact is too oblique (>60°)
            if (std::abs(delta_yaw) > M_PI / 3.0) {
                RCLCPP_WARN(this->get_logger(), 
                    "[STRAFE] BAD IMPACT! delta_yaw=%.1f° (>60°) — too rotated for swipe. Skipping to RETURN.",
                    delta_yaw * 180.0 / M_PI);
                bad_impact_ = true;
            }
            
            // Set forward velocity (lateral push is modulated later)
            swipe_vx_body_ = swipe_velocity_;
            swipe_vy_body_ = 0.0;

            RCLCPP_INFO(this->get_logger(), 
                "[STRAFE] Wall impact! Force: [fx=%.2f, fy=%.2f] -> swipe body vel: [vx=%.2f, vy=%.2f] | target_yaw: %.2f rad (curr: %.2f)",
                fx, fy, swipe_vx_body_, swipe_vy_body_, target_yaw_, current_yaw_);
        }
    }
    
}

void VioRecoveryFSM::heuristic_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (!direction_received_ && (msg->data == "LEFT" || msg->data == "RIGHT")) {
        strafe_direction_  = msg->data;
        direction_received_ = true;
        RCLCPP_INFO(this->get_logger(), "[STOP] Heuristic chose direction: %s", strafe_direction_.c_str());
    }
}

void VioRecoveryFSM::send_velocity(double vx_body, double vy_body, double vz_body, double omega_z) {
    geometry_msgs::msg::Twist vel;
    // Send open-loop velocity (feed-forward) to the controller
    vel.linear.x = vx_body;
    vel.linear.y = vy_body;
    vel.linear.z = vz_body;
    vel.angular.z = omega_z;
    pub_ff_cmd_vel_->publish(vel);
}

// ─────────────────────────────────────────────────────────────────────────────
// FSM loop
// ─────────────────────────────────────────────────────────────────────────────

void VioRecoveryFSM::fsm_loop() {
    // Publish current state for debugging
    std_msgs::msg::String state_msg;
    state_msg.data = state_to_string(current_state_);
    state_pub_->publish(state_msg);

    std_msgs::msg::Int32 state_num_msg;
    state_num_msg.data = static_cast<int>(current_state_);
    pub_fsm_state_num_->publish(state_num_msg);

    std_msgs::msg::Float64 yaw_msg;
    yaw_msg.data = recovery_start_yaw_;
    pub_target_yaw_->publish(yaw_msg);

    std_msgs::msg::String dir_msg;
    dir_msg.data = strafe_direction_;
    pub_strafe_direction_->publish(dir_msg);

    auto now_fsm = this->now();
    double elapsed = (now_fsm - state_entry_time_).seconds();
    
    // Calculate real dt based on timestamp to avoid rate assumptions
    double dt_fsm = (now_fsm - last_fsm_time_).seconds();
    last_fsm_time_ = now_fsm;
    
    // Guard against anomalous dt values
    if (dt_fsm <= 0.0 || dt_fsm > 0.5) dt_fsm = 0.1;

    switch (current_state_) {

        // ── NAVIGATE ──────────────────────────────────────────────────────────
        case DroneState::NAVIGATE: {
            break;
        }

        // ── STOP ──────────────────────────────────────────────────────────────
        case DroneState::STOP: {
            if (!stop_command_sent_) {
                // Switch move_manager to teleop mode and halt drone
                std_msgs::msg::String teleop_cmd;
                teleop_cmd.data = "teleop";
                pub_drone_cmd_->publish(teleop_cmd);

                std_msgs::msg::Bool teleop_active_msg;
                teleop_active_msg.data = true;
                pub_teleop_active_->publish(teleop_active_msg);

                send_velocity(0.0, 0.0, 0.0);

                stop_command_sent_ = true;
                RCLCPP_INFO(this->get_logger(), "[STOP] Sent STOP to drone + TELEOP to move_manager.");
            }

            // Drop ground marker after drone has settled
            if (stop_command_sent_ && !drop_command_sent_ && elapsed >= delay_before_drop_) {
                if (enable_bottom_) {
                    std_msgs::msg::Bool drop_msg;
                    drop_msg.data = true;
                    pub_drop_cmd_->publish(drop_msg);
                    RCLCPP_INFO(this->get_logger(), "[STOP] Dropped ground marker.");
                } else {
                    RCLCPP_INFO(this->get_logger(), "[STOP] Ground marker skipped (enable_bottom=false).");
                }
                drop_command_sent_ = true;
            }

            // Transition to STRAFE or RETURN once marker logic is handled
            if (drop_command_sent_) {
                if (!enable_lateral_) {
                    RCLCPP_INFO(this->get_logger(), "[STOP] → RETURN (enable_lateral=false). Skipping wall swipe.");
                    current_state_ = DroneState::RETURN;
                    state_entry_time_ = this->now();
                    return_start_y_ = odometry_received_ ? current_y_ : recovery_start_y_;
                    interp_cmd_yaw_received_ = false;
                } else if (direction_received_) {
                    RCLCPP_INFO(this->get_logger(), "[STOP] → STRAFE (%s)", strafe_direction_.c_str());
                    current_state_    = DroneState::STRAFE;
                    state_entry_time_ = this->now();
                } else if (elapsed >= (delay_before_drop_ + 5.0)) {
                    RCLCPP_WARN(this->get_logger(), "[STOP] No heuristic direction received. Defaulting to RIGHT.");
                    direction_received_ = true;
                }
            }
            break;
        }

        // ── STRAFE ────────────────────────────────────────────────────────────
        case DroneState::STRAFE: {
            if (!impact_detected_) {
                // Command open-loop lateral movement.
                // The vio_recovery_controller handles PI yaw alignment in parallel.
                double vy = (strafe_direction_ == "LEFT") ? strafe_velocity_ : -strafe_velocity_;
                send_velocity(0.0, vy, 0.0, 0.0);
            } else {
                // Impact detected: freeze lateral motion and stabilize
                strafe_duration_actual_ = (this->now() - state_entry_time_).seconds();
                send_velocity(0.0, 0.0, 0.0, 0.0);
                RCLCPP_INFO(this->get_logger(), "[STRAFE] → SETTLE (Waiting to stabilize). Strafe took %.2f s", strafe_duration_actual_);
                
                current_state_    = DroneState::SETTLE;
                state_entry_time_ = this->now();
            }

            // Abort if wall not found within timeout
            auto elapsed = (this->now() - state_entry_time_).seconds();
            if (elapsed >= strafe_timeout_) {
                RCLCPP_WARN(this->get_logger(), "[STRAFE] Timeout! No impact detected. Returning to center.");
                send_velocity(0.0, 0.0, 0.0, 0.0);
                current_state_    = DroneState::RETURN;
                state_entry_time_ = this->now();
                return_start_y_   = current_y_;
            }
            break;
        }

        // ── SETTLE ────────────────────────────────────────────────────────────
        case DroneState::SETTLE: {
            // Wait for physical stabilization AND yaw convergence to ~0°.
            // If yaw isn't near zero when SWIPE starts, body-frame vx will have
            // a world-frame component pushing into the wall (vx*sin(yaw_err)).
            auto now = this->now();
            auto elapsed = (now - state_entry_time_).seconds();

            // Compute yaw error (same logic as controller)
            double target_yaw_aligned = (std::cos(recovery_start_yaw_) > 0) ? 0.0 : M_PI;
            double yaw_err = current_yaw_ - target_yaw_aligned;
            while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
            while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
            bool yaw_converged = std::abs(yaw_err) < 0.02;  // ~1 degree

            if (elapsed >= 0.5 && yaw_converged) {
                if (bad_impact_) {
                    RCLCPP_WARN(this->get_logger(), "[SETTLE] Bad impact angle → skipping SWIPE, going to RETURN");
                    current_state_    = DroneState::RETURN;
                    state_entry_time_ = this->now();
                    return_start_y_   = current_y_;
                } else {
                    RCLCPP_INFO(this->get_logger(), "[SETTLE] Stabilized (yaw_err=%.3f rad) → SWIPE", yaw_err);
                    current_state_    = DroneState::SWIPE;
                    state_entry_time_ = this->now();
                }
            } else if (elapsed >= 3.0) {
                // Timeout: yaw didn't converge, go to SWIPE anyway but warn
                RCLCPP_WARN(this->get_logger(), "[SETTLE] Yaw didn't converge (err=%.3f rad, %.1f°). Proceeding to SWIPE.",
                    yaw_err, yaw_err * 180.0 / M_PI);
                if (bad_impact_) {
                    current_state_    = DroneState::RETURN;
                    return_start_y_   = current_y_;
                } else {
                    current_state_    = DroneState::SWIPE;
                }
                state_entry_time_ = this->now();
            } else {
                // Maintain position while PI yaw (in controller) corrects orientation
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[SETTLE] Waiting for yaw convergence (err=%.3f rad, %.1f°)",
                    yaw_err, yaw_err * 180.0 / M_PI);
                send_velocity(0.0, 0.0, 0.0, 0.0);
            }
            break;
        }

        // ── SWIPE ─────────────────────────────────────────────────────────────
        case DroneState::SWIPE: {
            if (!swipe_command_sent_) {
                // Trigger stain spawn on the wall
                std_msgs::msg::String swipe_msg;
                swipe_msg.data = strafe_direction_;
                pub_swipe_cmd_->publish(swipe_msg);
                swipe_command_sent_ = true;
                RCLCPP_INFO(this->get_logger(), "[SWIPE] Spawned swipe on %s wall.", strafe_direction_.c_str());
            }

            auto now = this->now();
            auto elapsed = (now - state_entry_time_).seconds();

            if (elapsed < swipe_duration_) {
                // Execute forward swipe along the wall (pure vx, no vy or yaw)
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[SWIPE] t=%.2f s | cmd: [vx=%.2f, vy=0.0, oz=0.0]",
                    elapsed, swipe_vx_body_);
                send_velocity(swipe_vx_body_, 0.0, 0.0, 0.0);
            } else if (elapsed < swipe_duration_ + 1.0) {
                // Kill momentum smoothly with a ramp-down
                double ramp = 1.0 - (elapsed - swipe_duration_) / 1.0;
                send_velocity(swipe_vx_body_ * ramp, 0.0, 0.0, 0.0);
            } else if (elapsed < swipe_duration_ + 2.0) {
                // Gently detach from wall to avoid trajectory friction during RETURN
                double vy_away = (strafe_direction_ == "LEFT") ? -0.05 : 0.05;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[SWIPE] Detaching from wall (vy=%.2f)", vy_away);
                send_velocity(0.0, vy_away, 0.0, 0.0);
            } else {
                // Transition to RETURN phase
                RCLCPP_INFO(this->get_logger(), "[SWIPE] → RETURN (corridor yaw: %.3f rad)", recovery_start_yaw_);
                current_state_    = DroneState::RETURN;
                state_entry_time_ = this->now();
                return_start_y_   = current_y_;
            }
            break;
        }

        // ── RETURN ────────────────────────────────────────────────────────────
        case DroneState::RETURN: {
            auto elapsed = (this->now() - state_entry_time_).seconds();

            double target_y;
            if (!enable_lateral_) {
                // We never went to the wall, so just stay exactly where we started the return phase
                target_y = return_start_y_;
            } else {
                // We want to travel return_distance_ AWAY from the wall.
                // If strafe was LEFT, the wall is at +Y, so we must go in -Y direction.
                // If strafe was RIGHT, the wall is at -Y, so we must go in +Y direction.
                target_y = return_start_y_ + ((strafe_direction_ == "LEFT") ? -return_distance_ : return_distance_);
            }
            
            // Current distance from the target
            double error_y = target_y - current_y_;
            bool at_center = std::abs(error_y) < 0.05;  // within 5cm of target

            // Timeout fallback: if odometry is drifted, don't stay here forever
            double max_return_duration = (return_velocity_ > 0.0)
                ? (return_distance_ / return_velocity_) + 5.0
                : 10.0;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[RETURN] error_y=%.3f m (target: |err|<0.05) | t=%.1f/%.1f s",
                error_y, elapsed, max_return_duration);

            if (!at_center && elapsed < max_return_duration) {
                // Move toward target Y position using closed-loop position feedback with proportional velocity
                double dist = std::abs(error_y);
                double vy_magnitude = std::clamp(dist * 1.5, 0.05, return_velocity_);
                
                // If error_y is positive (target > current), we need positive vy to go right (+Y).
                // If error_y is negative (target < current), we need negative vy to go left (-Y).
                double vy = (error_y > 0) ? vy_magnitude : -vy_magnitude;
                
                send_velocity(0.0, vy, 0.0, 0.0);
            } else {
                if (elapsed >= max_return_duration) {
                    RCLCPP_WARN(this->get_logger(), "[RETURN] Timeout! error_y=%.3f m. Proceeding to NAVIGATE anyway.", error_y);
                } else {
                    RCLCPP_INFO(this->get_logger(), "[RETURN] Reached target (error_y=%.3f m) → NAVIGATE", error_y);
                }

                if (!return_command_sent_) {
                    std_msgs::msg::String stop_msg;
                    stop_msg.data = "stop";
                    pub_drone_cmd_->publish(stop_msg);

                    std_msgs::msg::String stop_paint;
                    stop_paint.data = "STOP";
                    pub_swipe_cmd_->publish(stop_paint);

                    std_msgs::msg::Bool reset_drop;
                    reset_drop.data = false;
                    pub_drop_cmd_->publish(reset_drop);

                    return_command_sent_ = true;

                    // Reset all recovery flags
                    direction_received_   = false;
                    stop_command_sent_    = false;
                    drop_command_sent_    = false;
                    swipe_command_sent_   = false;
                    return_command_sent_  = false;
                    impact_detected_      = false;
                    bad_impact_           = false;

                    // Exit teleop mode so traj_interp can resume following path
                    std_msgs::msg::Bool teleop_msg;
                    teleop_msg.data = false;
                    pub_teleop_active_->publish(teleop_msg);

                    current_state_    = DroneState::NAVIGATE;
                    state_entry_time_ = this->now();
                    RCLCPP_INFO(this->get_logger(), "[RETURN] Finished → NAVIGATE");
                }
            }
            break;
        }
    }
}

std::string VioRecoveryFSM::state_to_string(DroneState state) {
    switch (state) {
        case DroneState::NAVIGATE: return "NAVIGATE";
        case DroneState::STOP:     return "STOP";
        case DroneState::STRAFE:   return "STRAFE";
        case DroneState::SETTLE:   return "SETTLE";
        case DroneState::SWIPE:    return "SWIPE";
        case DroneState::RETURN:   return "RETURN";
        default:                   return "UNKNOWN";
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VioRecoveryFSM>());
    rclcpp::shutdown();
    return 0;
}