#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "vio_recovery/msg/feature_count.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

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

        sub_fsm_state_ = this->create_subscription<std_msgs::msg::String>(
            "/fsm/current_state", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                fsm_is_navigating_ = (msg->data == "STOP");
            });

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ov_msckf/odomimu", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_x_ = msg->pose.pose.position.x;
                current_y_ = msg->pose.pose.position.y;
                
                double qx = msg->pose.pose.orientation.x;
                double qy = msg->pose.pose.orientation.y;
                double qz = msg->pose.pose.orientation.z;
                double qw = msg->pose.pose.orientation.w;
                current_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            });

        sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ov_msckf/points_slam", 10,
            std::bind(&SprayTargetHeuristic::points_callback, this, _1));

        eval_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&SprayTargetHeuristic::decide, this));

        last_decision_time_ = this->now();
    }

private:
    double max_reach_dist_;
    int min_feat_critical_;
    double max_marker_interval_;

    bool vio_in_danger_ = false;
    bool fsm_is_navigating_ = false;
    int feat_left_ = 100, feat_right_ = 100;
    
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    double last_spray_x_ = 0.0;
    
    double min_dist_left_ = 10.0;
    double min_dist_right_ = 10.0;
    
    bool last_spray_was_left_ = false;
    bool order_sent_ = false;
    rclcpp::Time last_decision_time_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_health_;
    rclcpp::Subscription<vio_recovery::msg::FeatureCount>::SharedPtr sub_features_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_fsm_state_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
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

    void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        
        double min_l = 10.0;
        double min_r = 10.0;

        for (; it_x != it_x.end(); ++it_x, ++it_y) {
            float x = *it_x;
            float y = *it_y;
            
            // Vector from drone to point
            double dx = x - current_x_;
            double dy = y - current_y_;
            
            // Rotate vector into drone's body frame
            double body_x = dx * std::cos(current_yaw_) + dy * std::sin(current_yaw_);
            double body_y = -dx * std::sin(current_yaw_) + dy * std::cos(current_yaw_);
            
            // Only consider points that are alongside the drone (+/- 2 meters ahead/behind)
            if (std::abs(body_x) < 2.0) {
                if (body_y > 0.1 && body_y < min_l) min_l = body_y;
                if (body_y < -0.1 && std::abs(body_y) < min_r) min_r = std::abs(body_y);
            }
        }
        
        min_dist_left_ = min_l;
        min_dist_right_ = min_r;
    }

    void decide() {
        std_msgs::msg::String order_msg;
        order_msg.data = "NONE";

        bool distance_trigger = (current_x_ - last_spray_x_) >= max_marker_interval_;

        if (vio_in_danger_ || distance_trigger) {

            // Only send orders if FSM is in STOP (waiting for our direction)
            if (!fsm_is_navigating_) {
                return;
            }

            std::string chosen_side = "NONE";
            
            // Choose the side based on dynamic point cloud distance first
            if (min_dist_left_ < 0.6 && min_dist_left_ < min_dist_right_ - 0.3) {
                chosen_side = "LEFT";
                RCLCPP_INFO(this->get_logger(), "Close to LEFT surface (dist=%.2f). Overriding feature heuristic.", min_dist_left_);
            } else if (min_dist_right_ < 0.6 && min_dist_right_ < min_dist_left_ - 0.3) {
                chosen_side = "RIGHT";
                RCLCPP_INFO(this->get_logger(), "Close to RIGHT surface (dist=%.2f). Overriding feature heuristic.", min_dist_right_);
            } else {
                // If somewhat centered, use the feature-based heuristic
                if (std::abs(feat_left_ - feat_right_) <= 10) {
                    chosen_side = last_spray_was_left_ ? "RIGHT" : "LEFT";
                } else if (feat_left_ < feat_right_) {
                    chosen_side = "LEFT";
                } else {
                    chosen_side = "RIGHT";
                }
            }

            if (!order_sent_) {
                // Publish just LEFT or RIGHT for the FSM
                order_msg.data = chosen_side;
                last_spray_was_left_ = (chosen_side == "LEFT");
                order_sent_ = true;
                last_decision_time_ = this->now();
                last_spray_x_ = current_x_;

                RCLCPP_WARN(this->get_logger(), "Decision: %s (vio_danger=%d, dist_trigger=%d)",
                    order_msg.data.c_str(), vio_in_danger_, distance_trigger);
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