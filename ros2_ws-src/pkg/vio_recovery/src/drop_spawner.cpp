#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class DropSpawnerNode : public rclcpp::Node {
public:
    DropSpawnerNode() : Node("drop_spawner"), is_spawned_(false),
                         current_x_(0.0), current_y_(0.0), current_z_(0.0), current_yaw_(0.0) {
        
        world_name_ = this->declare_parameter<std::string>("gazebo_world", "leonardo_race");

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos_odom = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Subscribe to Gazebo odometry instead of PX4, since this node is not needed in hardware
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/baby_k_0/odometry", qos_odom,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                
                // Gazebo Odometry is already ENU
                current_x_ = msg->pose.pose.position.x;
                current_y_ = msg->pose.pose.position.y;
                current_z_ = msg->pose.pose.position.z;

                Eigen::Quaterniond q(
                    msg->pose.pose.orientation.w,
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z
                );
                Eigen::Matrix3d R = q.toRotationMatrix();
                current_yaw_ = std::atan2(R(1, 0), R(0, 0));
            });

        // Subscribe to spawn command
        sub_spawn_cmd_ = this->create_subscription<std_msgs::msg::Bool>(
            "/command/drop_marker", 10, 
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data == true && !is_spawned_) {
                    spawn_rescue_target();
                    is_spawned_ = true;
                } else if (msg->data == false && is_spawned_) {
                    is_spawned_ = false;
                }
            });

        auto qos = rclcpp::QoS(1).transient_local();
        pub_aruco_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/aruco_rescue/pose", qos);
            
    }

private:
    void spawn_rescue_target() {
        
        // Dynamic calculation
        double tilt_angle_rad = 15.0 * (M_PI / 180.0);
        double backward_offset = std::abs(current_z_) * std::tan(tilt_angle_rad); 

        // Offset coordinates backwards
        double spawn_x = current_x_ - (backward_offset * std::cos(current_yaw_));
        double spawn_y = current_y_ - (backward_offset * std::sin(current_yaw_));
        double spawn_z = 0.015; // Slightly raised to avoid flickering with the floor
        
        // Orientation: Yaw only
        Eigen::Quaterniond q(Eigen::AngleAxisd(current_yaw_, Eigen::Vector3d::UnitZ()));

        // Publish pose to RViz
        geometry_msgs::msg::PoseStamped aruco_pose;
        aruco_pose.header.stamp = this->now();
        aruco_pose.header.frame_id = "map";
        aruco_pose.pose.position.x = spawn_x;
        aruco_pose.pose.position.y = spawn_y;
        aruco_pose.pose.position.z = spawn_z;
        aruco_pose.pose.orientation.x = q.x();
        aruco_pose.pose.orientation.y = q.y();
        aruco_pose.pose.orientation.z = q.z();
        aruco_pose.pose.orientation.w = q.w();
        pub_aruco_pose_->publish(aruco_pose);

        // Generate a unique name
        static int spawn_count = 1;
        char model_name[32];
        snprintf(model_name, sizeof(model_name), "splatter_down_%02d", spawn_count++);

        // Point to the new SDF file of the stain
        std::string template_path = "/root/ros2_ws/src/pkg/vio_recovery/models/splatter_stain/model.sdf";
        std::ifstream template_file(template_path);
        if (!template_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Cannot find SDF model at %s", template_path.c_str());
            return;
        }
        
        std::stringstream buffer;
        buffer << template_file.rdbuf();
        std::string sdf_string = buffer.str();
        template_file.close();

        // Replace only the model name, ignoring tags
        auto replace = [](std::string& str, const std::string& from, const std::string& to) {
            size_t pos = str.find(from);
            if (pos != std::string::npos) str.replace(pos, from.length(), to);
        };
        replace(sdf_string, "MODEL_NAME_PLACEHOLDER", std::string(model_name));

        // Save SDF
        std::string sdf_path = "/tmp/" + std::string(model_name) + ".sdf";
        std::ofstream sdf_file(sdf_path);
        if (sdf_file.is_open()) {
            sdf_file << sdf_string;
            sdf_file.close();
        }

        // Call service (removed /dev/null to see errors)
        std::string cmd = 
            "gz service -s /world/" + world_name_ + "/create"
            " --reqtype gz.msgs.EntityFactory"
            " --reptype gz.msgs.Boolean"
            " --timeout 5000"
            " --req \"sdf_filename: \\\"" + sdf_path + "\\\","
            " name: \\\"" + std::string(model_name) + "\\\","
            " pose: {position: {x: " + std::to_string(spawn_x) +
            ", y: " + std::to_string(spawn_y) +
            ", z: " + std::to_string(spawn_z) + "},"
            " orientation: {x: " + std::to_string(q.x()) + 
            ", y: " + std::to_string(q.y()) + 
            ", z: " + std::to_string(q.z()) + 
            ", w: " + std::to_string(q.w()) + "}}\" &";

        int ret = system(cmd.c_str());
        RCLCPP_INFO(this->get_logger(), "Spawned %s with ret %d", model_name, ret);
        
        RCLCPP_INFO(this->get_logger(), "Spawned stain %s at coordinates X:%.2f, Y:%.2f", model_name, spawn_x, spawn_y);
    }

    double current_x_, current_y_, current_z_, current_yaw_;
    bool is_spawned_; 
    std::string world_name_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_spawn_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_aruco_pose_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DropSpawnerNode>());
    rclcpp::shutdown();
    return 0;
}