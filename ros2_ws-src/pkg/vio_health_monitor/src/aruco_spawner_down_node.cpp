#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class ArucoSpawnerDownNode : public rclcpp::Node {
public:
    ArucoSpawnerDownNode() : Node("aruco_spawner_down_node"), is_spawned_(false),
                         current_x_(0.0), current_y_(0.0), current_z_(0.0), current_yaw_(0.0) {

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos_odom = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Riceve l'odometria da PX4 e calcola la posa nel mondo (ENU)
        sub_odom_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos_odom,
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                
                // Conversione diretta da NED (PX4) a ENU (Gazebo)
                current_x_ = msg->position[1];  // Est
                current_y_ = msg->position[0];  // Nord
                current_z_ = -msg->position[2]; // Alto

                // Calcolo dello Yaw
                Eigen::Quaterniond q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
                Eigen::Matrix3d R_ned_to_enu;
                R_ned_to_enu << 0, 1,  0,
                                1, 0,  0,
                                0, 0, -1;
                Eigen::Matrix3d R_frd_to_flu;
                R_frd_to_flu << 1,  0,  0,
                                0, -1,  0,
                                0,  0, -1;
                
                Eigen::Matrix3d R_body_enu = R_ned_to_enu * q_ned.toRotationMatrix() * R_frd_to_flu;
                current_yaw_ = std::atan2(R_body_enu(1, 0), R_body_enu(0, 0));
            });

        // Ascolta il comando di sparo da emergency_rescue
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
        
        // Calcolo dinamico: centratura perfetta basata sull'altezza reale del drone!
        double tilt_angle_rad = 15.0 * (M_PI / 180.0);
        double backward_offset = std::abs(current_z_) * std::tan(tilt_angle_rad); 

        // Coordinate sfalsate all'indietro per finire esattamente nel mirino
        double spawn_x = current_x_ - (backward_offset * std::cos(current_yaw_));
        double spawn_y = current_y_ - (backward_offset * std::sin(current_yaw_));
        double spawn_z = 0.015; // Leggermente rialzato per evitare sfarfallii col pavimento
        
        // Orientamento: Solo Yaw. Niente Pitch perché il "plane" SDF è già sdraiato a terra!
        Eigen::Quaterniond q(Eigen::AngleAxisd(current_yaw_, Eigen::Vector3d::UnitZ()));

        // Pubblica la posa su RViz
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

        // Generiamo un nome univoco per non far impazzire Gazebo
        static int spawn_count = 1;
        char model_name[32];
        snprintf(model_name, sizeof(model_name), "splatter_down_%02d", spawn_count++);

        // Punta al nuovo file SDF della macchia
        std::string template_path = "/root/ros2_ws/src/pkg/vio_health_monitor/models/splatter_stain/model.sdf";
        std::ifstream template_file(template_path);
        if (!template_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "ERRORE: Impossibile trovare il modello SDF in %s", template_path.c_str());
            return;
        }
        
        std::stringstream buffer;
        buffer << template_file.rdbuf();
        std::string sdf_string = buffer.str();
        template_file.close();

        // Rimpiazza solo il nome del modello, ignorando i tag
        auto replace = [](std::string& str, const std::string& from, const std::string& to) {
            size_t pos = str.find(from);
            if (pos != std::string::npos) str.replace(pos, from.length(), to);
        };
        replace(sdf_string, "MODEL_NAME_PLACEHOLDER", std::string(model_name));

        // Salva il file SDF temporaneo
        std::string sdf_path = "/tmp/" + std::string(model_name) + ".sdf";
        std::ofstream sdf_file(sdf_path);
        if (sdf_file.is_open()) {
            sdf_file << sdf_string;
            sdf_file.close();
        }

        // Chiamata al servizio di Gazebo
        std::string cmd = 
            "gz service -s /world/corridor/create"
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
            ", w: " + std::to_string(q.w()) + "}}\" > /dev/null 2>&1 &";

        system(cmd.c_str());
        
        RCLCPP_INFO(this->get_logger(), "Spawnata macchia %s alle coordinate X:%.2f, Y:%.2f", model_name, spawn_x, spawn_y);
    }

    double current_x_, current_y_, current_z_, current_yaw_;
    bool is_spawned_; 

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_spawn_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_aruco_pose_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoSpawnerDownNode>());
    rclcpp::shutdown();
    return 0;
}