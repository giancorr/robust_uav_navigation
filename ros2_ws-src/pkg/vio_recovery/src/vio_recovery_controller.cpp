#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class VioRecoveryController : public rclcpp::Node {
public:
    VioRecoveryController() : Node("vio_recovery_controller") {
        
        // Subscriptions
        sub_state_ = this->create_subscription<std_msgs::msg::Int32>(
            "/fsm/current_state_num", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                if (current_state_ != msg->data) {
                    // Reset integrals on state change
                    yaw_integral_ = 0.0;
                }
                current_state_ = msg->data;
            });

        sub_ff_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/recovery/ff_cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                ff_cmd_vel_ = *msg;
            });

        sub_target_yaw_ = this->create_subscription<std_msgs::msg::Float64>(
            "/recovery/target_yaw", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                target_yaw_ = msg->data;
            });

        sub_strafe_dir_ = this->create_subscription<std_msgs::msg::String>(
            "/recovery/strafe_direction", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                strafe_direction_ = msg->data;
            });

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/px4/odometry/out", qos,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                double qx = msg->pose.pose.orientation.x;
                double qy = msg->pose.pose.orientation.y;
                double qz = msg->pose.pose.orientation.z;
                double qw = msg->pose.pose.orientation.w;
                current_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            });

        sub_wrench_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/drone/external_wrench", 10,
            [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                current_fy_ = msg->wrench.force.y;
            });

        // Publisher
        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/teleop/velocity_increments", 10);

        last_time_ = this->now();
        timer_ = this->create_wall_timer(20ms, std::bind(&VioRecoveryController::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "VIO Recovery Controller Initialized (50Hz)");
    }

private:
    void control_loop() {
        double vy_feedback = 0.0;
        double omega_feedback = 0.0;

        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0 || dt > 0.1) dt = 0.02;

        // DroneState: 0=NAVIGATE, 1=STOP, 2=STRAFE, 3=SETTLE, 4=SWIPE, 5=RETURN

        // ── Yaw controller: active during STRAFE, SETTLE, and RETURN ──
        if (current_state_ == 2 || current_state_ == 3 || current_state_ == 5) {
            double target_yaw_aligned = (std::cos(target_yaw_) > 0) ? 0.0 : M_PI;
            double yaw_err = current_yaw_ - target_yaw_aligned;
            while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
            while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

            // STRAFE / SETTLE / RETURN: PI controller
            yaw_integral_ += yaw_err * dt;
            yaw_integral_ = std::clamp(yaw_integral_, -1.0, 1.0);

            double kp_yaw = (current_state_ == 5) ? 3.5 : 1.5;
            double ki_yaw = (current_state_ == 5) ? 1.0 : 0.2;
            omega_feedback = -(kp_yaw * yaw_err + ki_yaw * yaw_integral_);
            omega_feedback = std::clamp(omega_feedback, -1.5, 1.5);
            prev_yaw_err_ = yaw_err;  // Keep derivative smooth for transition
        } else {
            yaw_integral_ = 0.0;
            prev_yaw_err_ = 0.0;
        }

        // Only publish final command if not in NAVIGATE
        if (current_state_ != 0) {
            geometry_msgs::msg::Twist final_vel = ff_cmd_vel_;

            // Add yaw correction (PI for STRAFE/SETTLE/RETURN, 0 otherwise)
            final_vel.angular.z += omega_feedback;

            pub_vel_->publish(final_vel);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_state_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_ff_cmd_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_target_yaw_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_strafe_dir_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::TimerBase::SharedPtr timer_;

    int current_state_ = 0; // NAVIGATE
    geometry_msgs::msg::Twist ff_cmd_vel_;
    double target_yaw_ = 0.0;
    std::string strafe_direction_ = "RIGHT";
    double current_yaw_ = 0.0;
    double current_fy_ = 0.0;
    
    double yaw_integral_ = 0.0;
    double prev_yaw_err_ = 0.0;
    
    rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VioRecoveryController>());
    rclcpp::shutdown();
    return 0;
}
