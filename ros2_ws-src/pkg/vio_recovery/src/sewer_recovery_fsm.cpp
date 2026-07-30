#include "vio_recovery/sewer_recovery_fsm.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

SewerRecoveryFSM::SewerRecoveryFSM()
    : Node("sewer_recovery_fsm"),
      current_state_(SewerState::NAVIGATE),
      stop_command_sent_(false),
      recovery_start_x_(0.0), recovery_start_y_(0.0), recovery_start_z_(0.0), recovery_start_yaw_(0.0),
      current_x_(0.0), current_y_(0.0), current_z_(0.0), current_yaw_(0.0),
      odometry_received_(false),
      armed_(false),
      consecutive_consistent_count_(0)
{
    this->declare_parameter("enable_recovery", true);
    this->declare_parameter("trigger_on_potentially_inconsistent", false);
    this->declare_parameter("hover_timeout", 15.0);
    this->declare_parameter("ascend_velocity", 0.5);
    this->declare_parameter("min_consistent_to_arm", 10);

    enable_recovery_ = this->get_parameter("enable_recovery").as_bool();
    trigger_on_potentially_inconsistent_ = this->get_parameter("trigger_on_potentially_inconsistent").as_bool();
    hover_timeout_ = this->get_parameter("hover_timeout").as_double();
    ascend_velocity_ = this->get_parameter("ascend_velocity").as_double();
    min_consistent_to_arm_ = this->get_parameter("min_consistent_to_arm").as_int();

    sub_health_ = this->create_subscription<std_msgs::msg::String>(
        "/vio_health_status", 10, std::bind(&SewerRecoveryFSM::health_callback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/px4/odometry/out", rclcpp::SensorDataQoS(), std::bind(&SewerRecoveryFSM::odometry_callback, this, std::placeholders::_1));

    pub_move_manager_cmd_ = this->create_publisher<std_msgs::msg::String>("/move_manager/command", 10);
    pub_ff_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/recovery/ff_cmd_vel", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/fsm/current_state", 10);
    pub_fsm_state_num_ = this->create_publisher<std_msgs::msg::Int32>("/fsm/current_state_num", 10);
    pub_teleop_active_ = this->create_publisher<std_msgs::msg::Bool>("/move_manager/teleop_active", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&SewerRecoveryFSM::fsm_loop, this));

    node_start_time_ = this->now();
    last_fsm_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "SewerRecoveryFSM initialized (VERTICAL).");
}

std::string SewerRecoveryFSM::state_to_string(SewerState state) {
    switch(state) {
        case SewerState::NAVIGATE: return "NAVIGATE";
        case SewerState::HOVER: return "HOVER";
        case SewerState::ASCEND: return "ASCEND";
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

void SewerRecoveryFSM::health_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (!enable_recovery_) return;

    std::string health = msg->data;

    if (health == "CONSISTENT") {
        consecutive_consistent_count_++;
        if (consecutive_consistent_count_ >= min_consistent_to_arm_ && !armed_) {
            armed_ = true;
            RCLCPP_INFO(this->get_logger(), "FSM ARMED.");
        }
    } else {
        consecutive_consistent_count_ = 0;
    }

    if (!armed_) return;

    bool should_trigger = false;
    if (health == "INCONSISTENT") {
        should_trigger = true;
    } else if (trigger_on_potentially_inconsistent_ && health == "POTENTIALLY_INCONSISTENT") {
        should_trigger = true;
    }

    if (should_trigger && current_state_ == SewerState::NAVIGATE) {
        RCLCPP_WARN(this->get_logger(), "VIO FAIL DETECTED (%s)! Switching to HOVER.", health.c_str());
        current_state_ = SewerState::HOVER;
        state_entry_time_ = this->now();
        stop_command_sent_ = false;

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
    auto current_time = this->now();
    double dt = (current_time - last_fsm_time_).seconds();
    last_fsm_time_ = current_time;

    std_msgs::msg::String state_msg;
    state_msg.data = state_to_string(current_state_);
    state_pub_->publish(state_msg);

    std_msgs::msg::Int32 state_num_msg;
    state_num_msg.data = static_cast<int>(current_state_);
    pub_fsm_state_num_->publish(state_num_msg);

    std_msgs::msg::Bool teleop_msg;

    switch (current_state_) {
        case SewerState::NAVIGATE: {
            teleop_msg.data = false;
            pub_teleop_active_->publish(teleop_msg);
            break;
        }

        case SewerState::HOVER: {
            teleop_msg.data = true;
            pub_teleop_active_->publish(teleop_msg);

            if (!stop_command_sent_) {
                std_msgs::msg::String stop_msg;
                stop_msg.data = "STOP";
                pub_move_manager_cmd_->publish(stop_msg);
                stop_command_sent_ = true;
            }

            // Command zero velocity to hover. 
            // In a real scenario, we might want a slow yaw spin here.
            send_velocity(0.0, 0.0, 0.0, 0.2); // Slow yaw spin to find features

            double elapsed = (current_time - state_entry_time_).seconds();
            if (elapsed >= hover_timeout_) {
                RCLCPP_ERROR(this->get_logger(), "HOVER timeout! Switching to ASCEND.");
                current_state_ = SewerState::ASCEND;
                state_entry_time_ = current_time;
            }
            break;
        }

        case SewerState::ASCEND: {
            teleop_msg.data = true;
            pub_teleop_active_->publish(teleop_msg);

            // Climb out of the sewer
            send_velocity(0.0, 0.0, ascend_velocity_, 0.0);
            
            // Ascend indefinitely until node is killed, or we could add a condition to check Z height.
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
