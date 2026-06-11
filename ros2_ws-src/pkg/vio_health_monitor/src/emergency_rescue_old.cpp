#include "vio_health_monitor/emergency_rescue_old.hpp"

EmergencyRescueFSM::EmergencyRescueFSM() : Node("emergency_rescue_node"),
    current_state_(DroneState::TAKEOFF), vio_warning_(false), mission_started_(false),
    initial_yaw_captured_(false), odom_received_(false), has_sprayed_once_(false), hover_triggered_(false),
    surface_dist_(-1.0), surface_sector_(-1.0), target_sector_("NONE"), takeoff_yaw_target_(0.0),
    takeoff_x_(0.0), takeoff_y_(0.0), nav_step_(0), current_maneuver_time_(0.0), spray_cooldown_(10.0),
    target_yaw_lock_(0.0), hold_x_(0.0), hold_y_(0.0), hold_z_(0.0), hold_yaw_(0.0),
    last_approach_dist_(0.0), approach_start_x_(0.0), approach_start_y_(0.0),
    features_left_(0), features_right_(0), min_features_threshold_(50),
    dist_left_(-1.0), dist_center_(-1.0), dist_right_(-1.0),
    omega_max_(0.1), alpha_max_(0.02), enable_spray_(true) {

    this->declare_parameter<bool>("is_sim", true);
    this->declare_parameter<double>("target_x", 10.0);
    this->declare_parameter<double>("target_y", 0.0);
    this->declare_parameter<double>("target_z", 2.0);
    this->declare_parameter<double>("target_yaw", 1.57); 
    this->declare_parameter<double>("max_wall_dist", 4.0);
    this->declare_parameter<double>("spray_dist", 0.50); 
    this->declare_parameter<double>("max_marker_interval", 10.0);
    this->declare_parameter<double>("recovery_time", 3.0);
    this->declare_parameter<double>("spray_duration", 1.0);
    this->declare_parameter<double>("cruise_speed", 1.0);
    this->declare_parameter<double>("acceleration", 0.8);
    this->declare_parameter<int>("min_features_threshold", 50);
    this->declare_parameter<double>("omega_max", 0.1);   
    this->declare_parameter<double>("alpha_max", 0.02);   
    this->declare_parameter<bool>("enable_spray", true);
    this->declare_parameter<bool>("is_downcam", false);

    this->get_parameter("is_sim", is_sim_);
    this->get_parameter("max_wall_dist", max_wall_dist_);
    this->get_parameter("spray_dist", spray_dist_);
    this->get_parameter("max_marker_interval", max_marker_interval_);
    this->get_parameter("recovery_time", recovery_time_);
    this->get_parameter("spray_duration", spray_duration_);
    this->get_parameter("cruise_speed", cruise_speed_);
    this->get_parameter("acceleration", acceleration_);
    this->get_parameter("min_features_threshold", min_features_threshold_);
    this->get_parameter("omega_max", omega_max_);   
    this->get_parameter("alpha_max", alpha_max_); 
    this->get_parameter("enable_spray", enable_spray_);

    sub_health_ = this->create_subscription<std_msgs::msg::String>(
        "/vio_health_status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            if (msg->data == "INCONSISTENT" || msg->data == "POTENTIALLY_INCONSISTENT") {
                vio_warning_ = true;
            } else if (msg->data == "CONSISTENT") {
                vio_warning_ = false;
            }
        });

    sub_features_ = this->create_subscription<vio_health_monitor::msg::FeatureCount>(
        "/vio_help/feature_distribution", 10,
        [this](const vio_health_monitor::msg::FeatureCount::SharedPtr msg) {
            features_left_ = msg->left_count;
            features_right_ = msg->right_count;
        });

    sub_surface_ = this->create_subscription<vio_health_monitor::msg::SurfaceInfo>(
        "/surface_info", 10,
        [this](const vio_health_monitor::msg::SurfaceInfo::SharedPtr msg) {
            dist_left_ = msg->dist_left;
            dist_center_ = msg->dist_center;
            dist_right_ = msg->dist_right;
        });

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    sub_odom_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos,
        [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            current_x_ = msg->position[0];
            current_y_ = msg->position[1];
            current_z_ = msg->position[2];
            double qw = msg->q[0];
            double qx = msg->q[1];
            double qy = msg->q[2];
            double qz = msg->q[3];
            current_yaw_ = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
            
            odom_history_.push_back({current_x_, current_y_, current_yaw_});
            if (odom_history_.size() > 3) {
                odom_history_.pop_front();
            }
            odom_received_ = true; 
        });

    pub_move_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/fsm/trajectory_command", 10);
    pub_spray_cmd_ = this->create_publisher<std_msgs::msg::Bool>("/hardware/spray_cmd", 10);
    pub_spawn_aruco_ = this->create_publisher<std_msgs::msg::Bool>("/command/spawn_aruco", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/fsm/current_state", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&EmergencyRescueFSM::fsm_loop, this));

    start_time_ = this->now();
    takeoff_time_ = this->now();
    last_spray_time_ = this->now();
    last_approach_time_ = this->now();
    returning_start_time_ = this->now();
}

double EmergencyRescueFSM::calculate_trajectory_duration(double distance, double v_max, double a_max) {
    if (distance <= 0.05) return 1.0;
    double dist_to_accel_decel = (v_max * v_max) / a_max;
    if (distance > dist_to_accel_decel) {
        double t_accel = v_max / a_max;
        double dist_cruise = distance - dist_to_accel_decel;
        double t_cruise = dist_cruise / v_max;
        return (2.0 * t_accel) + t_cruise;
    } else {
        return 2.0 * std::sqrt(distance / a_max);
    }
}

double EmergencyRescueFSM::calculate_rotation_duration(double angle_rad, double omega_max, double alpha_max) {
    double angle = std::abs(angle_rad);
    if (angle < 0.01) return 1.0;
    double angle_to_accel_decel = (omega_max * omega_max) / alpha_max;
    if (angle > angle_to_accel_decel) {
        double t_accel = omega_max / alpha_max;
        double angle_cruise = angle - angle_to_accel_decel;
        double t_cruise = angle_cruise / omega_max;
        return (2.0 * t_accel) + t_cruise;
    } else {
        return 2.0 * std::sqrt(angle / alpha_max);
    }
}

void EmergencyRescueFSM::send_movement(double x, double y, double z, double yaw, double duration) {
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = {x, y, z, 0.0, 0.0, yaw, duration};
    pub_move_cmd_->publish(cmd);
}

double EmergencyRescueFSM::distance_from_last_spray() {
    double dx = current_x_ - last_spray_position_x_;
    double dy = current_y_ - last_spray_position_y_;
    return std::sqrt(dx*dx + dy*dy);
}

bool EmergencyRescueFSM::has_any_valid_wall() {
    bool left_valid   = (dist_left_   > 0.0 && dist_left_   <= max_wall_dist_);
    bool right_valid  = (dist_right_  > 0.0 && dist_right_  <= max_wall_dist_);
    bool center_valid = (dist_center_ > 0.0 && dist_center_ <= max_wall_dist_);
    return left_valid || right_valid || center_valid;
}

bool EmergencyRescueFSM::should_spray_now() {
    bool has_wall_now = has_any_valid_wall();
    bool far_enough   = (!has_sprayed_once_) || (distance_from_last_spray() > 1.0);
    bool time_enough  = (!has_sprayed_once_) || ((this->now() - last_spray_time_).seconds() > spray_cooldown_);

    int total_features = features_left_ + features_right_;
    bool low_features = (total_features > 0) && (total_features < min_features_threshold_);

    bool vio_trigger      = (vio_warning_ || low_features) && has_wall_now && far_enough && time_enough;
    bool interval_trigger = has_sprayed_once_ && (distance_from_last_spray() >= max_marker_interval_) && has_wall_now && time_enough;

    return enable_spray_ && (vio_trigger || interval_trigger);
}

std::string EmergencyRescueFSM::state_to_string(DroneState state) {
    switch (state) {
        case DroneState::TAKEOFF:             return "TAKEOFF";
        case DroneState::NAVIGATING:          return "NAVIGATING";
        case DroneState::BRAKING:             return "BRAKING";
        case DroneState::PIVOTING_TO_SURFACE: return "PIVOTING_TO_SURFACE";
        case DroneState::APPROACHING_SURFACE: return "APPROACHING_SURFACE";
        case DroneState::SPRAYING:            return "SPRAYING";
        case DroneState::RECOVERING:          return "RECOVERING";
        case DroneState::RETURNING:           return "RETURNING";
        default:                              return "UNKNOWN";
    }
}

void EmergencyRescueFSM::fsm_loop() {
    if (!initial_yaw_captured_) {
        if (odom_received_ && (this->now() - start_time_).seconds() > 2.0) {
            takeoff_yaw_target_ = current_yaw_;
            takeoff_x_ = current_x_;
            takeoff_y_ = current_y_;
            initial_yaw_captured_ = true;
            last_spray_time_ = this->now();
        }
        return; 
    }

    std_msgs::msg::String state_msg;
    state_msg.data = state_to_string(current_state_);
    state_pub_->publish(state_msg);

    switch (current_state_) {
        case DroneState::TAKEOFF:
            {
                double target_z_abs = std::abs(this->get_parameter("target_z").as_double());
                double t_z = -target_z_abs; 
                
                if (nav_step_ == 0) { 
                    if (!mission_started_) {
                        double z_error = std::abs(current_z_ - t_z);
                        double duration = std::max(z_error / 0.5, 2.0); 
                        send_movement(current_x_, current_y_, t_z, current_yaw_, duration);
                        mission_started_ = true;
                        takeoff_time_ = this->now();
                    }
                    bool z_reached = std::abs(current_z_ - t_z) < 0.1;
                    bool timeout   = (this->now() - takeoff_time_).seconds() > 8.0;
                    if (mission_started_ && (z_reached || timeout)) {
                        mission_started_ = false; 
                        nav_step_ = 1; 
                        last_approach_time_ = this->now();
                    }
                } 
                else if (nav_step_ == 1) { 
                    if (!mission_started_) {
                        send_movement(current_x_, current_y_, current_z_, current_yaw_, 2.0);
                        mission_started_ = true;
                    }
                    if ((this->now() - last_approach_time_).seconds() > 2.0) {
                        takeoff_yaw_target_ = current_yaw_; 
                        takeoff_x_ = current_x_; 
                        takeoff_y_ = current_y_;
                        last_spray_time_ = this->now(); 
                        vio_warning_ = false; 
                        mission_started_ = false;
                        nav_step_ = 0; 
                        current_state_ = DroneState::NAVIGATING;
                    }
                }
            }
            break;

        case DroneState::NAVIGATING:
            {
                double t_x = this->get_parameter("target_x").as_double();
                double t_y = this->get_parameter("target_y").as_double();
                double t_z = -std::abs(this->get_parameter("target_z").as_double());
                
                if (nav_step_ == 0) {                               
                    if (!mission_started_) {
                        double dx = t_x - current_x_;
                        double dy = t_y - current_y_;
                        double dist_to_target = std::sqrt(dx*dx + dy*dy);
                        double dynamic_dur = dist_to_target / cruise_speed_;
                        
                        send_movement(t_x, t_y, t_z, takeoff_yaw_target_, dynamic_dur);
                        mission_started_ = true; 
                    }
                } 
                else if (nav_step_ == 1 && !mission_started_) {     
                    double target_yaw = this->get_parameter("target_yaw").as_double();
                    double scan_yaw = std::atan2(
                        std::sin(takeoff_yaw_target_ + target_yaw), 
                        std::cos(takeoff_yaw_target_ + target_yaw));
                    double yaw_diff = std::abs(std::atan2(
                        std::sin(scan_yaw - current_yaw_),
                        std::cos(scan_yaw - current_yaw_)));
                    double rot_dur = calculate_rotation_duration(yaw_diff, omega_max_, alpha_max_);
                    rot_dur = std::max(rot_dur, 3.0);
                    send_movement(t_x, t_y, current_z_, scan_yaw, rot_dur); 
                    mission_started_ = true;
                }

                if (should_spray_now()) {   
                    bool left_wall_valid  = (dist_left_  > 0.0 && dist_left_  <= max_wall_dist_);
                    bool right_wall_valid = (dist_right_ > 0.0 && dist_right_ <= max_wall_dist_);
                    static bool last_spray_was_left = false;
                    bool prefer_left = false;
                    
                    if (std::abs(features_left_ - features_right_) <= 5) {
                        prefer_left = !last_spray_was_left;
                    } else {
                        prefer_left = (features_left_ < features_right_);
                    }

                    if (prefer_left) {
                        if (left_wall_valid)       { target_sector_ = "LEFT";  last_spray_was_left = true;  }
                        else if (right_wall_valid) { target_sector_ = "RIGHT"; last_spray_was_left = false; }
                        else                         target_sector_ = "CENTER";
                    } else {
                        if (right_wall_valid)      { target_sector_ = "RIGHT"; last_spray_was_left = false; }
                        else if (left_wall_valid)  { target_sector_ = "LEFT";  last_spray_was_left = true;  }
                        else                         target_sector_ = "CENTER";
                    }
                    
                    double natural_stop_dist = (cruise_speed_ * cruise_speed_) / (2.0 * acceleration_);
                    double stop_dist = std::max(natural_stop_dist, 0.3);
                    
                    double correzione_laterale = 0.0;
                    if (left_wall_valid && right_wall_valid) {
                        double errore_centraggio = dist_left_ - dist_right_;
                        double K_p = 0.3; 
                        correzione_laterale = errore_centraggio * K_p;
                    } else if (left_wall_valid && !right_wall_valid) {
                        double errore = dist_left_ - 1.0; 
                        correzione_laterale = errore * 0.3;
                    } else if (!left_wall_valid && right_wall_valid) {
                        double errore = 1.0 - dist_right_; 
                        correzione_laterale = errore * 0.3;
                    }

                    double fwd_dx = stop_dist * std::cos(takeoff_yaw_target_);
                    double fwd_dy = stop_dist * std::sin(takeoff_yaw_target_);

                    double lat_dx = -correzione_laterale * std::sin(takeoff_yaw_target_);
                    double lat_dy = correzione_laterale * std::cos(takeoff_yaw_target_);

                    return_x_ = current_x_ + lat_dx; 
                    return_y_ = current_y_ + lat_dy; 
                    return_z_ = current_z_; 
                    return_yaw_ = takeoff_yaw_target_; 
                    
                    approach_start_x_ = current_x_ + fwd_dx + lat_dx; 
                    approach_start_y_ = current_y_ + fwd_dy + lat_dy; 
                    
                    send_movement(approach_start_x_, approach_start_y_, return_z_, return_yaw_, 3.0);
                    last_approach_time_ = this->now(); 
                    current_maneuver_time_ = 1.5; 
                    current_state_ = DroneState::BRAKING; 
                    mission_started_ = false;
                }
            }
            break;

        case DroneState::BRAKING:
            {
                double elapsed = (this->now() - last_approach_time_).seconds();
                if (elapsed > current_maneuver_time_) {
                    mission_started_ = false; 
                    last_approach_time_ = this->now(); 
                    
                    if (this->get_parameter("is_downcam").as_bool()) {
                        hold_x_ = current_x_; 
                        hold_y_ = current_y_; 
                        hold_z_ = current_z_; 
                        hold_yaw_ = current_yaw_;
                        target_yaw_lock_ = current_yaw_;
                        spraying_start_time_ = this->now(); 
                        current_state_ = DroneState::SPRAYING;
                    } else {
                        current_state_ = DroneState::PIVOTING_TO_SURFACE; 
                    }
                }
            }
            break;

        case DroneState::PIVOTING_TO_SURFACE:
            {
                double max_search_angle = 40.0 * (M_PI / 180.0); 

                if (!mission_started_) {
                    target_yaw_lock_ = return_yaw_;
                    if (target_sector_ == "LEFT")       target_yaw_lock_ -= max_search_angle; 
                    else if (target_sector_ == "RIGHT") target_yaw_lock_ += max_search_angle; 
                    target_yaw_lock_ = std::atan2(std::sin(target_yaw_lock_), std::cos(target_yaw_lock_));

                    double pivot_duration = calculate_rotation_duration(max_search_angle, omega_max_, alpha_max_);
                    pivot_duration = std::max(pivot_duration, 3.0);

                    send_movement(approach_start_x_, approach_start_y_, return_z_, target_yaw_lock_, pivot_duration);
                    
                    mission_started_ = true; 
                    last_approach_time_ = this->now();
                    current_maneuver_time_ = pivot_duration;
                }

                double yaw_diff        = current_yaw_ - return_yaw_;
                double normalized_diff = std::atan2(std::sin(yaw_diff), std::cos(yaw_diff));
                double yaw_turned      = std::abs(normalized_diff);
                double min_lock_yaw    = 30.0 * (M_PI / 180.0); 
                double max_safe_yaw    = 45.0 * (M_PI / 180.0); 
                bool distance_valid    = (dist_center_ >= spray_dist_ && dist_center_ <= max_wall_dist_);

                if (yaw_turned >= min_lock_yaw && distance_valid) {
                    target_yaw_lock_ = current_yaw_; 
                    
                    send_movement(current_x_, current_y_, return_z_, target_yaw_lock_, 1.0); 
                    
                    mission_started_ = false; 
                    current_state_ = DroneState::APPROACHING_SURFACE; 
                    break;
                }
                
                bool search_failed = (yaw_turned > max_safe_yaw) || 
                                     ((this->now() - last_approach_time_).seconds() > current_maneuver_time_ + 2.0);
                if (search_failed) {
                    send_movement(approach_start_x_, approach_start_y_, return_z_, return_yaw_, 3.0);
                    returning_start_time_ = this->now(); 
                    current_maneuver_time_ = 3.0; 
                    mission_started_ = false; 
                    current_state_ = DroneState::RETURNING; 
                    break;
                }
            }
            break;

        case DroneState::APPROACHING_SURFACE:
            {
                if (!mission_started_) {
                    if (dist_center_ > 0.0) {
                        double dist_to_cover = dist_center_ - spray_dist_;
                        if (dist_to_cover > 0.05) { 
                            approach_start_x_ = current_x_; 
                            approach_start_y_ = current_y_;
                            hold_x_ = current_x_ + dist_to_cover * std::cos(target_yaw_lock_); 
                            hold_y_ = current_y_ + dist_to_cover * std::sin(target_yaw_lock_);
                            double duration = calculate_trajectory_duration(dist_to_cover, 0.2, 0.15); 
                            send_movement(hold_x_, hold_y_, return_z_, target_yaw_lock_, duration);
                            mission_started_ = true; 
                            last_approach_time_ = this->now(); 
                            current_maneuver_time_ = duration; 
                        } 
                        else {
                            last_approach_dist_ = std::sqrt(
                                std::pow(current_x_ - approach_start_x_, 2) + 
                                std::pow(current_y_ - approach_start_y_, 2));
                            hold_x_   = current_x_; 
                            hold_y_   = current_y_; 
                            hold_z_   = current_z_; 
                            hold_yaw_ = current_yaw_;
                            spraying_start_time_ = this->now(); 
                            mission_started_ = false; 
                            current_state_ = DroneState::SPRAYING;
                        }
                    }
                } 
                else {
                    double elapsed = (this->now() - last_approach_time_).seconds();
                    if (elapsed > current_maneuver_time_) {
                        mission_started_ = false; 
                    }
                }
            }
            break;

        case DroneState::SPRAYING:
            {
                bool is_downcam = this->get_parameter("is_downcam").as_bool();
                // Definiamo il tempo di attesa qui, parametrizzabile o fisso
                double post_spray_wait = is_downcam ? 3.0 : 0.5;

                if (!mission_started_) {
                    double total_time = spray_duration_ + post_spray_wait;
                    send_movement(hold_x_, hold_y_, hold_z_, hold_yaw_, total_time);
                    spraying_start_time_ = this->now(); 
                    mission_started_ = true; 
                    nav_step_ = 0;
                }

                double elapsed_since_start = (this->now() - spraying_start_time_).seconds();
                
                if (nav_step_ == 0 && elapsed_since_start > 0.5) {
                    std_msgs::msg::Bool m; 
                    m.data = true;
                    if (is_sim_) { pub_spawn_aruco_->publish(m); } 
                    else         { pub_spray_cmd_->publish(m);    }
                    last_spray_time_ = this->now(); 
                    nav_step_ = 1;
                }
                
                if (nav_step_ == 1 && (this->now() - last_spray_time_).seconds() > spray_duration_) {
                    std_msgs::msg::Bool m; 
                    m.data = false;
                    if (is_sim_) { pub_spawn_aruco_->publish(m); } 
                    else         { pub_spray_cmd_->publish(m);    }
                    
                    if (is_downcam) {
                        double offset = 0.15; 
                        double strafe_x = hold_x_ - offset * std::sin(hold_yaw_);
                        double strafe_y = hold_y_ + offset * std::cos(hold_yaw_);
                        send_movement(strafe_x, strafe_y, hold_z_, hold_yaw_, post_spray_wait);
                    }
                    nav_step_ = 2;
                }
                
                double elapsed_since_spray = (this->now() - last_spray_time_).seconds();
                if (nav_step_ == 2 && elapsed_since_spray > post_spray_wait) {
                    last_spray_position_x_ = current_x_; 
                    last_spray_position_y_ = current_y_;
                    vio_warning_ = false; 
                    has_sprayed_once_ = true; 
                    mission_started_ = false; 
                    
                    if (is_downcam) {
                        nav_step_ = 0; 
                        last_spray_time_ = this->now();
                        current_state_ = DroneState::NAVIGATING;
                    } else {
                        recovering_start_time_ = this->now(); 
                        current_state_ = DroneState::RECOVERING;
                    }
                }
            }
            break;

        case DroneState::RECOVERING:
            {
                if (!mission_started_) {
                    double step = 0.15; 
                    double recovery_x = hold_x_ + step * std::cos(target_yaw_lock_);
                    double recovery_y = hold_y_ + step * std::sin(target_yaw_lock_);
                    send_movement(recovery_x, recovery_y, hold_z_, target_yaw_lock_, recovery_time_);
                    recovering_start_time_ = this->now();
                    mission_started_ = true;
                }
                if ((this->now() - recovering_start_time_).seconds() > recovery_time_) {
                    mission_started_ = false;
                    current_state_ = DroneState::RETURNING;
                }
            }
            break;

        case DroneState::RETURNING:
            {
                if (!mission_started_) {
                    double L = 0.5;
                    double final_ret_x = approach_start_x_ - L * std::cos(return_yaw_);
                    double final_ret_y = approach_start_y_ - L * std::sin(return_yaw_);
                    double diff_x = final_ret_x - hold_x_;
                    double diff_y = final_ret_y - hold_y_;
                    double dist_to_safe_point = std::sqrt(diff_x*diff_x + diff_y*diff_y);
                    double rec_duration = calculate_trajectory_duration(dist_to_safe_point, 0.2, 0.15);
                    rec_duration = std::max(rec_duration, 2.5); 
                    send_movement(final_ret_x, final_ret_y, return_z_, return_yaw_, rec_duration);
                    returning_start_time_ = this->now(); 
                    current_maneuver_time_ = rec_duration; 
                    mission_started_ = true; 
                }
                if ((this->now() - returning_start_time_).seconds() >= current_maneuver_time_) {
                    mission_started_ = false; 
                    nav_step_ = 0; 
                    last_spray_time_ = this->now(); 
                    vio_warning_ = false; 
                    current_state_ = DroneState::NAVIGATING;
                }
            }
            break;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmergencyRescueFSM>());
    rclcpp::shutdown();
    return 0;
}