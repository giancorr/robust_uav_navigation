#include "vio_recovery/sewer_recovery_fsm.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

SewerRecoveryFSM::SewerRecoveryFSM()
    : Node("sewer_recovery_fsm"),
      current_state_(SewerState::NAVIGATE),
      stop_command_sent_(false),
      impact_detected_(false),
      recovery_start_x_(0.0), recovery_start_y_(0.0), recovery_start_z_(0.0), recovery_start_yaw_(0.0),
      current_x_(0.0), current_y_(0.0), current_z_(0.0), current_yaw_(0.0),
      odometry_received_(false),
      fsm_enabled_(false),
      armed_(false),
      consecutive_consistent_count_(0)
{
    this->declare_parameter("enable_recovery", true);
    this->declare_parameter("trigger_on_potentially_inconsistent", false);
    this->declare_parameter("approach_velocity", 0.1);
    this->declare_parameter("impact_force_threshold", 0.5);
    this->declare_parameter("swipe_velocity", 0.2);
    this->declare_parameter("swipe_duration", 2.0);
    this->declare_parameter("return_velocity", 0.15);
    this->declare_parameter("return_duration", 2.0);
    this->declare_parameter("hover_timeout", 2.0);
    this->declare_parameter("settle_duration", 1.5);
    this->declare_parameter("min_consistent_to_arm", 10);
    this->declare_parameter("arming_delay", 15.0);

    enable_recovery_ = this->get_parameter("enable_recovery").as_bool();
    trigger_on_potentially_inconsistent_ = this->get_parameter("trigger_on_potentially_inconsistent").as_bool();
    approach_velocity_ = this->get_parameter("approach_velocity").as_double();
    impact_force_threshold_ = this->get_parameter("impact_force_threshold").as_double();
    swipe_velocity_ = this->get_parameter("swipe_velocity").as_double();
    swipe_duration_ = this->get_parameter("swipe_duration").as_double();
    return_velocity_ = this->get_parameter("return_velocity").as_double();
    return_duration_ = this->get_parameter("return_duration").as_double();
    hover_timeout_ = this->get_parameter("hover_timeout").as_double();
    settle_duration_ = this->get_parameter("settle_duration").as_double();
    min_consistent_to_arm_ = this->get_parameter("min_consistent_to_arm").as_int();
    arming_delay_ = this->get_parameter("arming_delay").as_double();

    sub_health_ = this->create_subscription<std_msgs::msg::String>(
        "/vio_health_status", 10, std::bind(&SewerRecoveryFSM::health_callback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/px4/odometry/out", rclcpp::SensorDataQoS(), std::bind(&SewerRecoveryFSM::odometry_callback, this, std::placeholders::_1));
    sub_wrench_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/drone/external_wrench", 10, std::bind(&SewerRecoveryFSM::wrench_callback, this, std::placeholders::_1));

    pub_move_manager_cmd_ = this->create_publisher<std_msgs::msg::String>("/move_manager/command", 10);
    pub_ff_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/recovery/ff_cmd_vel", 10);
    pub_swipe_cmd_ = this->create_publisher<std_msgs::msg::String>("/command/swipe_paint", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/fsm/current_state", 10);
    swipe_command_sent_ = false;
    pub_fsm_state_num_ = this->create_publisher<std_msgs::msg::Int32>("/fsm/current_state_num", 10);
    pub_teleop_active_ = this->create_publisher<std_msgs::msg::Bool>("/move_manager/teleop_active", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&SewerRecoveryFSM::fsm_loop, this));

    sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
        "/fsm/enable_arming", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data && !fsm_enabled_) {
                RCLCPP_INFO(this->get_logger(), "FSM Arming Enabled by external node.");
                enable_time_ = this->now();
            }
            fsm_enabled_ = msg->data;
        });

    node_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    enable_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    last_fsm_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    state_entry_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    RCLCPP_INFO(this->get_logger(), "SewerRecoveryFSM initialized (TUBE CRAWLER). Waiting for /fsm/enable_arming");
}

std::string SewerRecoveryFSM::state_to_string(SewerState state) {
    switch(state) {
        case SewerState::NAVIGATE: return "NAVIGATE";
        case SewerState::SETTLE: return "SETTLE";
        case SewerState::APPROACH: return "APPROACH";
        case SewerState::SWIPE: return "SWIPE";
        case SewerState::RETURN: return "RETURN";
        case SewerState::HOVER: return "HOVER";
        default: return "UNKNOWN";
    }
}

void SewerRecoveryFSM::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);
    odometry_received_ = true;
}

void SewerRecoveryFSM::wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    if (current_state_ == SewerState::APPROACH) {
        double force_magnitude = std::sqrt(
            msg->wrench.force.x * msg->wrench.force.x +
            msg->wrench.force.y * msg->wrench.force.y
        );
        if (force_magnitude >= impact_force_threshold_) {
            RCLCPP_INFO(this->get_logger(), "[APPROACH] Impact detected! Force: %.2f N", force_magnitude);
            impact_detected_ = true;
        }
    }
}

void SewerRecoveryFSM::health_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (!enable_recovery_) return;

    if (node_start_time_.seconds() == 0.0 && this->now().seconds() > 0.0) {
        node_start_time_ = this->now();
        last_fsm_time_ = this->now();
        state_entry_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Clock started. node_start_time_ set to %.2f", node_start_time_.seconds());
    }

    if (node_start_time_.seconds() == 0.0 || !odometry_received_) {
        return;
    }

    // FSM must be explicitly enabled (e.g. by reaching sewer_entry)
    if (!fsm_enabled_) {
        return;
    }

    // Enforce initial timeout so the FSM ignores noisy VIO right after takeoff
    if ((this->now() - enable_time_).seconds() < arming_delay_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Ignoring VIO health: waiting for arming delay (%.1f / %.1f s)",
            (this->now() - enable_time_).seconds(), arming_delay_);
        return;
    }

    // Only arm if in NAVIGATE
    if (current_state_ != SewerState::NAVIGATE) {
        return;
    }

    std::string health = msg->data;

    if (health == "CONSISTENT" || 
        (!trigger_on_potentially_inconsistent_ && health == "POTENTIALLY_INCONSISTENT") ||
        health == "POTENTIALLY_CONSISTENT") {
        
        if (health == "CONSISTENT") {
            consecutive_consistent_count_++;
        } else if (health == "POTENTIALLY_CONSISTENT" || health == "POTENTIALLY_INCONSISTENT") {
            // Recovering from INCONSISTENT or facing noisy environment:
            // count very conservatively (no decrement, no increment)
            // Just hold and wait for full CONSISTENT before arming again
        }

        if (consecutive_consistent_count_ >= min_consistent_to_arm_ && !armed_) {
            armed_ = true;
            RCLCPP_INFO(this->get_logger(), "FSM ARMED after %d consistent readings.", consecutive_consistent_count_);
        }
        return;
    }

    if (!armed_) {
        return;
    }

    bool should_trigger = false;
    if (health == "INCONSISTENT" || (trigger_on_potentially_inconsistent_ && health == "POTENTIALLY_INCONSISTENT")) {
        should_trigger = true;
        armed_ = false;
        consecutive_consistent_count_ = 0;
    }

    if (should_trigger && current_state_ == SewerState::NAVIGATE) {
        RCLCPP_WARN(this->get_logger(), "VIO FAIL DETECTED (%s)! Switching to SETTLE.", health.c_str());
        current_state_ = SewerState::SETTLE;
        state_entry_time_ = this->now();
        stop_command_sent_ = false;
        impact_detected_ = false;
        swipe_command_sent_ = false;

        recovery_start_x_ = current_x_;
        recovery_start_y_ = current_y_;
        recovery_start_z_ = current_z_;
        recovery_start_yaw_ = current_yaw_;
    } else if (health == "CONSISTENT" && current_state_ == SewerState::HOVER) {
        RCLCPP_INFO(this->get_logger(), "VIO RECOVERED in HOVER! Resuming NAVIGATE.");
        current_state_ = SewerState::NAVIGATE;
        stop_command_sent_ = false;
    }
}

void SewerRecoveryFSM::send_velocity(double vx, double vy, double vz, double omega_z) {
    geometry_msgs::msg::Twist vel_msg;
    vel_msg.linear.x = vx;
    vel_msg.linear.y = vy;
    vel_msg.linear.z = vz;
    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = omega_z;
    pub_ff_cmd_vel_->publish(vel_msg);
}

void SewerRecoveryFSM::fsm_loop() {
    if (node_start_time_.seconds() == 0.0 && this->now().seconds() > 0.0) {
        node_start_time_ = this->now();
        last_fsm_time_ = this->now();
        state_entry_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Clock started (fsm_loop). node_start_time_ set to %.2f", node_start_time_.seconds());
    }

    if (node_start_time_.seconds() == 0.0) return;

    auto current_time = this->now();
    last_fsm_time_ = current_time;

    std_msgs::msg::String state_msg;
    state_msg.data = state_to_string(current_state_);
    state_pub_->publish(state_msg);

    std_msgs::msg::Int32 state_num_msg;
    state_num_msg.data = static_cast<int>(current_state_);
    pub_fsm_state_num_->publish(state_num_msg);

    std_msgs::msg::Bool teleop_msg;
    if (current_state_ != SewerState::NAVIGATE) {
        teleop_msg.data = true;
        pub_teleop_active_->publish(teleop_msg);
        
        if (!stop_command_sent_) {
            std_msgs::msg::String stop_msg;
            stop_msg.data = "STOP";
            pub_move_manager_cmd_->publish(stop_msg);
            stop_command_sent_ = true;
        }
    } else {
        teleop_msg.data = false;
        pub_teleop_active_->publish(teleop_msg);
    }

    double elapsed = (current_time - state_entry_time_).seconds();

    switch (current_state_) {
        case SewerState::NAVIGATE: {
            break;
        }

        case SewerState::SETTLE: {
            // Wait for physical stabilization
            send_velocity(0.0, 0.0, 0.0, 0.0);

            if (elapsed >= settle_duration_) {
                RCLCPP_INFO(this->get_logger(), "[SETTLE] Stabilized → APPROACH");
                current_state_ = SewerState::APPROACH;
                state_entry_time_ = current_time;
                impact_detected_ = false;
            }
            break;
        }

        case SewerState::APPROACH: {
            // Move forward (+x) until we hit the wall
            send_velocity(approach_velocity_, 0.0, 0.0, 0.0);

            if (impact_detected_) {
                RCLCPP_INFO(this->get_logger(), "[APPROACH] → SWIPE");
                current_state_ = SewerState::SWIPE;
                state_entry_time_ = current_time;
            } else if (elapsed > 30.0) {
                // Safety timeout
                RCLCPP_ERROR(this->get_logger(), "[APPROACH] Timeout! Forcing HOVER.");
                current_state_ = SewerState::HOVER;
                state_entry_time_ = current_time;
            }
            break;
        }

        case SewerState::SWIPE: {
            if (elapsed < 0.3) {
                // Phase 1 (first 0.3s): retract from wall to cancel the position integrator overshoot.
                // If we command vx=0 immediately, the PID holds the position inside the wall.
                // Sending a brief opposite velocity unwinds the integrator for a softer impact.
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                    "[SWIPE] Retracting from wall (vx=%.2f, t=%.2fs)", -approach_velocity_, elapsed);
                send_velocity(-approach_velocity_, 0.0, 0.0, 0.0);
            } else if (elapsed < swipe_duration_ + 0.3) {
                // Phase 2: Execute the vertical swipe
                if (!swipe_command_sent_) {
                    std_msgs::msg::String swipe_msg;
                    swipe_msg.data = "FRONT";
                    pub_swipe_cmd_->publish(swipe_msg);
                    swipe_command_sent_ = true;
                    RCLCPP_INFO(this->get_logger(), "[SWIPE] Spawned swipe on FRONT wall.");
                }
                send_velocity(0.0, 0.0, -swipe_velocity_, 0.0);
            } else if (elapsed < swipe_duration_ + 1.3) {
                // Phase 3: Kill vertical momentum smoothly with a ramp-down (1.0 second)
                double ramp = 1.0 - (elapsed - (swipe_duration_ + 0.3)) / 1.0;
                send_velocity(0.0, 0.0, -swipe_velocity_ * ramp, 0.0);
            } else if (elapsed < swipe_duration_ + 2.3) {
                // Phase 4: Gently detach from wall to avoid trajectory friction during RETURN (1.0 second)
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                    "[SWIPE] Detaching from wall (vx=%.2f)", -0.05);
                send_velocity(-0.05, 0.0, 0.0, 0.0);
            } else {
                // Transition to RETURN
                RCLCPP_INFO(this->get_logger(), "[SWIPE] → RETURN");
                current_state_ = SewerState::RETURN;
                state_entry_time_ = current_time;
            }
            break;
        }

        case SewerState::RETURN: {
            // Move backward (-x) away from the wall
            send_velocity(-return_velocity_, 0.0, 0.0, 0.0);

            if (elapsed >= return_duration_) {
                std_msgs::msg::String stop_paint;
                stop_paint.data = "STOP";
                pub_swipe_cmd_->publish(stop_paint);

                RCLCPP_INFO(this->get_logger(), "[RETURN] → HOVER");
                current_state_ = SewerState::HOVER;
                state_entry_time_ = current_time;
            }
            break;
        }

        case SewerState::HOVER: {
            // Command zero velocity to stabilize
            send_velocity(0.0, 0.0, 0.0, 0.0);

            if (elapsed >= hover_timeout_) {
                RCLCPP_ERROR(this->get_logger(), "[HOVER] Timeout reached. Forcing emergency APPROACH.");
                current_state_ = SewerState::APPROACH;
                state_entry_time_ = current_time;
                impact_detected_ = false;
                swipe_command_sent_ = false;
            }
            break;
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SewerRecoveryFSM>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
