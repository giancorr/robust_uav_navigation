// emergency_rescue.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp> 
#include <vio_health_monitor/msg/surface_info.hpp>
#include <vio_health_monitor/msg/feature_count.hpp>
#include <cmath>
#include <algorithm>
#include <deque>
#include <tuple>
#include <string>

enum class DroneState {
    TAKEOFF,
    NAVIGATING,
    BRAKING,
    PIVOTING_TO_SURFACE,
    APPROACHING_SURFACE,
    SPRAYING,
    RECOVERING,
    RETURNING
};

class EmergencyRescueFSM : public rclcpp::Node {
public:
    EmergencyRescueFSM();

private:
    // --- Internal Logic Methods ---
    double calculate_trajectory_duration(double distance, double v_max, double a_max);
    double calculate_rotation_duration(double angle_rad, double omega_max, double alpha_max);
    void send_movement(double x, double y, double z, double yaw, double duration);
    double distance_from_last_spray();
    bool has_any_valid_wall();
    bool should_spray_now();
    void fsm_loop();
    std::string state_to_string(DroneState state);

    // --- State Variables ---
    DroneState current_state_;
    bool is_sim_, vio_warning_, mission_started_, initial_yaw_captured_, odom_received_, has_sprayed_once_, hover_triggered_;
    double surface_dist_, surface_sector_;
    std::string target_sector_;
    double omega_max_;
    double alpha_max_;

    // Odometry and Target
    double current_x_, current_y_, current_z_, current_yaw_;
    double takeoff_yaw_target_, takeoff_x_, takeoff_y_;
    double return_x_, return_y_, return_z_, return_yaw_;
    double last_spray_position_x_, last_spray_position_y_;

    // FSM Variables
    int nav_step_;
    double current_maneuver_time_, spray_cooldown_, target_yaw_lock_;
    double hold_x_, hold_y_, hold_z_, hold_yaw_;
    double max_wall_dist_, spray_dist_, max_marker_interval_, recovery_time_, spray_duration_, cruise_speed_, acceleration_;
    double last_approach_dist_, approach_start_x_, approach_start_y_;
    bool enable_spray_;

    // Sensors
    int features_left_, features_right_;
    int min_features_threshold_;  // NEW: minimum feature threshold for proactive spray
    double dist_left_, dist_center_, dist_right_;

    // Odometry History and Timers
    std::deque<std::tuple<double, double, double>> odom_history_;
    rclcpp::Time start_time_, takeoff_time_, recovering_start_time_, last_approach_time_, returning_start_time_, last_spray_time_, spraying_start_time_;

    // --- ROS Nodes ---
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_health_;
    rclcpp::Subscription<vio_health_monitor::msg::SurfaceInfo>::SharedPtr sub_surface_;
    rclcpp::Subscription<vio_health_monitor::msg::FeatureCount>::SharedPtr sub_features_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_move_cmd_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_spray_cmd_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_spawn_aruco_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};