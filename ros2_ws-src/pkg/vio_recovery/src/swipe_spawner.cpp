#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class SwipeSpawnerNode : public rclcpp::Node {
public:
    SwipeSpawnerNode() : Node("swipe_spawner"), current_side_("NONE"),
                         current_x_(0.0), current_y_(0.0), current_z_(0.0), current_yaw_(0.0) {
        
        this->declare_parameter<std::string>("gazebo_world", "sewer");
        this->get_parameter("gazebo_world", world_name_);

        this->declare_parameter<double>("impact_wall_distance", 0.35);
        this->get_parameter("impact_wall_distance", impact_wall_distance_);

        this->declare_parameter<std::string>("odom_topic", "/model/baby_k_0/odometry");
        std::string odom_topic;
        this->get_parameter("odom_topic", odom_topic);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos_odom = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, qos_odom,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {

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

        // Sub to spawn command
        sub_spawn_cmd_ = this->create_subscription<std_msgs::msg::String>(
            "/command/swipe_paint", 10, 
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data == "LEFT" || msg->data == "RIGHT" || msg->data == "FRONT") {
                    current_side_ = msg->data;
                    spawn_paint_stroke();
                }
            });

        auto qos = rclcpp::QoS(1).transient_local();
        pub_aruco_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/aruco_rescue/pose", qos);
    }

private:
    void spawn_paint_stroke() {
        double spawn_x, spawn_y, spawn_z;
        Eigen::Quaterniond q;

        if (world_name_ == "sewer") {
            // Sewer case: vertical swipe on front wall
            spawn_x = current_x_ + impact_wall_distance_;
            spawn_y = current_y_;
            spawn_z = current_z_ - 0.5; // shift down to cover the swipe

            // Rotate by -pi/2 around Y so that X (length 1.5) goes to Z axis and faces -X
            q = Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()));
        } else {
            // Corridor case: horizontal swipe on side wall
            double swipe_length = 1.0; 
            spawn_x = current_x_ + (swipe_length / 2.0);
            spawn_y = (current_side_ == "LEFT") ? 0.97 : -0.97;
            spawn_z = current_z_ - 0.1;

            double marker_yaw = 0.0;
            if (current_side_ == "LEFT") {
                marker_yaw = M_PI;
            }

            // Orientation: Yaw only (with pitch offset)
            Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(marker_yaw, Eigen::Vector3d::UnitZ()));
            Eigen::Quaterniond q_pitch(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));
            q = q_yaw * q_pitch; 
        }

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

        RCLCPP_INFO(this->get_logger(), "Spawning mesh at X: %.2f, Y: %.2f, Z: %.2f", spawn_x, spawn_y, spawn_z);

        // Generate a unique name
        static int spawn_count = 1;
        char model_name[32];
        snprintf(model_name, sizeof(model_name), "paint_stroke_%02d", spawn_count++);

        // Point to the new SDF file of the stain
        std::string template_path = "/root/ros2_ws/src/pkg/vio_recovery/models/aruco_rescue/model.sdf";
        std::ifstream template_file(template_path);
        if (!template_file.is_open()) return;
        
        std::stringstream buffer;
        buffer << template_file.rdbuf();
        std::string sdf_string = buffer.str();
        template_file.close();

        // Replace only the model name, ignoring tags
        size_t pos = sdf_string.find("MODEL_NAME_PLACEHOLDER");
        if (pos != std::string::npos) {
            sdf_string.replace(pos, std::string("MODEL_NAME_PLACEHOLDER").length(), std::string(model_name));
        }

        // Save SDF
        std::string sdf_path = "/tmp/" + std::string(model_name) + ".sdf";
        std::ofstream sdf_file(sdf_path);
        if (sdf_file.is_open()) {
            sdf_file << sdf_string;
            sdf_file.close();
        }

        // Call service
        std::string cmd = "gz service -s /world/" + world_name_ + "/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 --req \"sdf_filename: \\\"" + sdf_path + "\\\", name: \\\"" + std::string(model_name) + "\\\", pose: {position: {x: " + std::to_string(spawn_x) + ", y: " + std::to_string(spawn_y) + ", z: " + std::to_string(spawn_z) + "}, orientation: {x: " + std::to_string(q.x()) + ", y: " + std::to_string(q.y()) + ", z: " + std::to_string(q.z()) + ", w: " + std::to_string(q.w()) + "}}\" > /dev/null 2>&1 &";
        system(cmd.c_str());
    }

    double current_x_, current_y_, current_z_, current_yaw_;
    double impact_wall_distance_;
    std::string current_side_;
    std::string world_name_;
    std::string model_path_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_spawn_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_aruco_pose_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwipeSpawnerNode>());
    rclcpp::shutdown();
    return 0;
}