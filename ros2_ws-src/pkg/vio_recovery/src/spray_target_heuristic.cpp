#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "vio_recovery/msg/feature_count.hpp"
#include "vio_recovery/msg/surface_info.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>

using std::placeholders::_1;

class SprayTargetHeuristic : public rclcpp::Node
{
public:
    SprayTargetHeuristic() : Node("spray_heuristic_node")
    {
        this->declare_parameter<double>("max_reachable_dist", 3.0); 
        this->declare_parameter<int>("min_features_critical", 30);
        this->declare_parameter<double>("max_marker_interval", 5.0);

        max_reach_dist_ = this->get_parameter("max_reachable_dist").as_double();
        min_feat_critical_ = this->get_parameter("min_features_critical").as_int();
        max_marker_interval_ = this->get_parameter("max_marker_interval").as_double();

        decision_pub_ = this->create_publisher<std_msgs::msg::String>("/decision/spray_target", 10);

        sub_health_ = this->create_subscription<std_msgs::msg::String>(
            "/vio_health_status", 10, std::bind(&SprayTargetHeuristic::health_callback, this, _1));

        sub_features_ = this->create_subscription<vio_recovery::msg::FeatureCount>(
            "/vio_help/feature_distribution", 10, std::bind(&SprayTargetHeuristic::feature_callback, this, _1));

        sub_surface_ = this->create_subscription<vio_recovery::msg::SurfaceInfo>(
            "/surface_info", 10, std::bind(&SprayTargetHeuristic::surface_callback, this, _1));

        sub_fsm_state_ = this->create_subscription<std_msgs::msg::String>(
            "/fsm/current_state", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                fsm_is_navigating_ = (msg->data == "NAVIGATING");
            });

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        sub_odom_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                current_x_ = msg->position[0];
            });

        eval_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&SprayTargetHeuristic::evaluate_situation, this));

        last_decision_time_ = this->now();
    }

private:
    double max_reach_dist_;
    int min_feat_critical_;
    double max_marker_interval_;

    bool vio_in_danger_ = false;
    bool fsm_is_navigating_ = false;
    int feat_left_ = 100, feat_right_ = 100;
    double dist_left_ = 10.0, dist_right_ = 10.0;
    
    double current_x_ = 0.0;
    double last_spray_x_ = 0.0;
    
    bool last_spray_was_left_ = false;
    bool order_sent_ = false;
    rclcpp::Time last_decision_time_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_health_;
    rclcpp::Subscription<vio_recovery::msg::FeatureCount>::SharedPtr sub_features_;
    rclcpp::Subscription<vio_recovery::msg::SurfaceInfo>::SharedPtr sub_surface_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_fsm_state_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
    rclcpp::TimerBase::SharedPtr eval_timer_;

    void health_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "INCONSISTENT" || msg->data == "POTENTIALLY_INCONSISTENT") {
            if (!vio_in_danger_) {
                RCLCPP_WARN(this->get_logger(), "VIO DANGER DETECTED: %s", msg->data.c_str());
            }
            vio_in_danger_ = true;
        } 
        else if (msg->data == "CONSISTENT" || msg->data == "POTENTIALLY_CONSISTENT") {
            if (vio_in_danger_) {
                RCLCPP_INFO(this->get_logger(), "VIO returned to safe state: %s", msg->data.c_str());
            }
            vio_in_danger_ = false;
        }
        else {
            RCLCPP_WARN_ONCE(this->get_logger(), "Received unknown VIO state: %s", msg->data.c_str());
        }
    }

    void feature_callback(const vio_recovery::msg::FeatureCount::SharedPtr msg) {
        feat_left_ = msg->left_count;
        feat_right_ = msg->right_count;
    }

    void surface_callback(const vio_recovery::msg::SurfaceInfo::SharedPtr msg) {
        dist_left_ = msg->dist_left;
        dist_right_ = msg->dist_right;
    }

    void evaluate_situation() {
        std_msgs::msg::String order_msg;
        order_msg.data = "NONE";

        bool distance_trigger = (current_x_ - last_spray_x_) >= max_marker_interval_;

        if (vio_in_danger_ || distance_trigger) {

            // Only send orders if FSM is in NAVIGATING
            if (!fsm_is_navigating_) {
                return;
            }

            bool left_valid = (dist_left_ > 0.0 && dist_left_ < max_reach_dist_);
            bool right_valid = (dist_right_ > 0.0 && dist_right_ < max_reach_dist_);
            std::string chosen_side = "NONE";
            
            RCLCPP_INFO(this->get_logger(), "Surface evaluation - Dist L: %.2f (valid: %d), Dist R: %.2f (valid: %d), Max Reach: %.2f", 
                        dist_left_, left_valid, dist_right_, right_valid, max_reach_dist_);

            // If both surfaces are reachable, choose the one with less features
            // If features are balanced, alternate sides
            // In each case, drop
            if (left_valid && right_valid) {
                if (std::abs(feat_left_ - feat_right_) <= 10) {
                    chosen_side = last_spray_was_left_ ? "RIGHT" : "LEFT";
                } else if (feat_left_ < feat_right_) {
                    chosen_side = "LEFT";
                } else {
                    chosen_side = "RIGHT";
                }
            } else if (left_valid) {
                chosen_side = "LEFT";
            } else if (right_valid) {
                chosen_side = "RIGHT";
            }

            if (!order_sent_) {
                if (chosen_side != "NONE") {
                    order_msg.data = "DROP_THEN_STAMP_" + chosen_side;
                    last_spray_was_left_ = (chosen_side == "LEFT");
                } else {
                    order_msg.data = "DROP_ONLY";
                }
                order_sent_ = true;
                last_decision_time_ = this->now();
                last_spray_x_ = current_x_;

                if (distance_trigger && !vio_in_danger_) {
                    RCLCPP_WARN(this->get_logger(), "Distance interval exceeded (%.1fm). Order: %s", max_marker_interval_, order_msg.data.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Order sent: %s", order_msg.data.c_str());
                }
            } else if ((this->now() - last_decision_time_).seconds() > 15.0) {
                order_sent_ = false;
            }
        } else {
            if ((this->now() - last_decision_time_).seconds() > 8.0) {
                order_sent_ = false;
            }
        }

        if (order_msg.data != "NONE") {
            decision_pub_->publish(order_msg);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SprayTargetHeuristic>());
    rclcpp::shutdown();
    return 0;
}