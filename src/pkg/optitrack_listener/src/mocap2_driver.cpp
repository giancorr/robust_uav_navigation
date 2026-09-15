#include <iostream>
#include <sstream>
#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <optional>

#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/node_interfaces/node_logging.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

#include <Eigen/Core>
#include "utils.h"

using std::placeholders::_1;
using std::placeholders::_2;


class OptitrackDriverNode : public rclcpp::Node
{
private:

NatNetClient * client;

sNatNetClientConnectParams client_params;
sServerDescription server_description;
sDataDescriptions * data_descriptions{nullptr};
sFrameOfMocapData latest_data;
sRigidBodyData latest_body_frame_data;

// Vecchio publisher singolo rimosso, ora usiamo un vettore di publisher
// rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ot_odom_pub_;
std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> ot_odom_pubs_;

// Struttura per memorizzare le informazioni di ogni corpo rigido
struct BodyInfo
{
    int64_t id;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub;
    std::string child_frame_id;
};
std::vector<BodyInfo> body_infos_;

std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

//params
std::string odom_frame_id_;  // prefisso per i frame child
std::string map_frame_id_;
bool publish_tf_;

std::string connection_type_;
std::string server_address_;
std::string local_address_;
std::string multicast_address_;
uint16_t server_command_port_;
uint16_t server_data_port_;

// Lista degli ID dei corpi rigidi da pubblicare
std::vector<int64_t> rigid_body_ids_;

uint32_t frame_number_{0};

public:
    OptitrackDriverNode(): Node("mocap2_driver"){

        declare_parameter<std::string>("connection_type", "Unicast");
        declare_parameter<std::string>("server_address", "000.000.000.000");
        declare_parameter<std::string>("local_address", "000.000.000.000");
        declare_parameter<std::string>("multicast_address", "000.000.000.000");
        declare_parameter<uint16_t>("server_command_port", 0);
        declare_parameter<uint16_t>("server_data_port", 0);

        declare_parameter<std::string>("odom_frame_id", "ot_odom"); 
        declare_parameter<std::string>("map_frame_id", "map"); 
        declare_parameter<bool>("publish_tf", true);

        // Nuovo parametro: lista di ID dei corpi rigidi (default [0])
        declare_parameter<std::vector<int64_t>>("rigid_body_ids", std::vector<int64_t>{0});
      
        get_parameter<std::string>("connection_type", connection_type_);
        get_parameter<std::string>("server_address", server_address_);
        get_parameter<std::string>("local_address", local_address_);
        get_parameter<std::string>("multicast_address", multicast_address_);
        get_parameter<uint16_t>("server_command_port", server_command_port_);
        get_parameter<uint16_t>("server_data_port", server_data_port_);

        odom_frame_id_ = get_parameter("odom_frame_id").as_string();
        map_frame_id_ = get_parameter("map_frame_id").as_string();
        publish_tf_ = get_parameter("publish_tf").as_bool();

        get_parameter("rigid_body_ids", rigid_body_ids_);
        if (rigid_body_ids_.empty()) {
            RCLCPP_WARN(get_logger(), "La lista rigid_body_ids è vuota. Nessun corpo verrà pubblicato.");
        }

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Creazione di un publisher per ogni ID richiesto
        for (int64_t id : rigid_body_ids_) {
            std::string topic_name = "/optitrack/body_" + std::to_string(id) + "/odometry";
            auto pub = this->create_publisher<nav_msgs::msg::Odometry>(topic_name, qos);
            std::string child_frame = odom_frame_id_ + "_body_" + std::to_string(id);
            body_infos_.push_back({id, pub, child_frame});
            RCLCPP_INFO(get_logger(), "Publisher creato per il corpo ID %ld sul topic %s", id, topic_name.c_str());
        }

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


        client = new NatNetClient();
        client->SetFrameReceivedCallback(process_frame_callback, this);


    }

    ~OptitrackDriverNode()
    {
        if (client) {
            client->Disconnect();
            delete client;
        }
    }

    // Callback statica che corrisponde alla firma richiesta dalla NatNet SDK
    static void NATNET_CALLCONV process_frame_callback(sFrameOfMocapData* data, void* pUserData)
    {
        static_cast<OptitrackDriverNode*>(pUserData)->process_frame(data);
    }
    
    void set_settings_optitrack()
    {
        if (connection_type_ == "Multicast") {
            client_params.connectionType = ConnectionType::ConnectionType_Multicast;
            client_params.multicastAddress = multicast_address_.c_str();
        } else if (connection_type_ == "Unicast") {
            client_params.connectionType = ConnectionType::ConnectionType_Unicast;
        } else {
            RCLCPP_FATAL(get_logger(), "Unknown connection type -- options are Multicast, Unicast");
            rclcpp::shutdown();
        }

        client_params.serverAddress = server_address_.c_str();
        client_params.localAddress = local_address_.c_str();
        client_params.serverCommandPort = server_command_port_;
        client_params.serverDataPort = server_data_port_;
    }

    std::chrono::nanoseconds get_optitrack_system_latency(sFrameOfMocapData * data)
    {
    const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

    if (bSystemLatencyAvailable) {
        const double clientLatencySec =
        client->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp);
        const double clientLatencyMillisec = clientLatencySec * 1000.0;
        const double transitLatencyMillisec =
        client->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;

        const double largeLatencyThreshold = 100.0;
        if (clientLatencyMillisec >= largeLatencyThreshold) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *this->get_clock(), 500,
            "Optitrack system latency >%.0f ms: [Transmission: %.0fms, Total: %.0fms]",
            largeLatencyThreshold, transitLatencyMillisec, clientLatencyMillisec);
        }

        return round<std::chrono::nanoseconds>(std::chrono::duration<float>{clientLatencySec});
    } else {
        RCLCPP_WARN_ONCE(get_logger(), "Optitrack's system latency not available");
        return std::chrono::nanoseconds::zero();
    }
    }

    // Funzione helper per trovare un corpo rigido per ID
    const sRigidBodyData* find_rigid_body_by_id(sFrameOfMocapData* data, int64_t id)
    {
        for (int i = 0; i < data->nRigidBodies; ++i) {
            if (data->RigidBodies[i].ID == id) {
                return &data->RigidBodies[i];
            }
        }
        return nullptr;
    }

    void process_frame(sFrameOfMocapData * data)
    {
        frame_number_++;
        rclcpp::Duration frame_delay = rclcpp::Duration(get_optitrack_system_latency(data));

        // Scorriamo i corpi che ci interessano
        for (const auto& body_info : body_infos_) {
            const sRigidBodyData* body_data = find_rigid_body_by_id(data, body_info.id);
            if (!body_data) {
                // Non presente in questo frame, saltiamo
                continue;
            }

            nav_msgs::msg::Odometry ros_odom;
            ros_odom.header.stamp = now() - frame_delay;
            ros_odom.header.frame_id = map_frame_id_;
            ros_odom.child_frame_id = body_info.child_frame_id;

            ros_odom.pose.pose.position.x = body_data->x;
            ros_odom.pose.pose.position.y = body_data->y;
            ros_odom.pose.pose.position.z = body_data->z;
            ros_odom.pose.pose.orientation.x = body_data->qx;
            ros_odom.pose.pose.orientation.y = body_data->qy;
            ros_odom.pose.pose.orientation.z = body_data->qz;
            ros_odom.pose.pose.orientation.w = body_data->qw;

            // Pubblicazione sul topic dedicato
            body_info.pub->publish(ros_odom);

            if (publish_tf_) {
                geometry_msgs::msg::TransformStamped tf_odom;
                utilities::tf_from_odom(tf_odom, ros_odom); // usa child_frame_id interno
                tf_broadcaster_->sendTransform(tf_odom);
            }
        }

        // Opzionale: avviso se non sono stati trovati corpi (solo se la lista non è vuota)
        if (!body_infos_.empty() && data->nRigidBodies == 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "Nessun corpo rigido ricevuto in questo frame.");
        }
    }

    bool connect_optitrack()
    {
      RCLCPP_INFO(
        get_logger(),
        "Trying to connect to Optitrack NatNET SDK at %s ...", server_address_.c_str());

      client->Disconnect();
      set_settings_optitrack();

      if (client->Connect(client_params) == ErrorCode::ErrorCode_OK) {
        RCLCPP_INFO(get_logger(), "... connected!");

        memset(&server_description, 0, sizeof(server_description));
        client->GetServerDescription(&server_description);
        if (!server_description.HostPresent) {
          RCLCPP_DEBUG(get_logger(), "Unable to connect to server. Host not present.");
          return false;
        }

        if (client->GetDataDescriptionList(&data_descriptions) != ErrorCode_OK || !data_descriptions) {
          RCLCPP_DEBUG(get_logger(), "[Client] Unable to retrieve Data Descriptions.\n");
        }

        RCLCPP_INFO(get_logger(), "\n[Client] Server application info:\n");
        RCLCPP_INFO(
          get_logger(), "Application: %s (ver. %d.%d.%d.%d)\n",
          server_description.szHostApp, server_description.HostAppVersion[0],
          server_description.HostAppVersion[1], server_description.HostAppVersion[2],
          server_description.HostAppVersion[3]);
        RCLCPP_INFO(
          get_logger(), "NatNet Version: %d.%d.%d.%d\n", server_description.NatNetVersion[0],
          server_description.NatNetVersion[1],
          server_description.NatNetVersion[2], server_description.NatNetVersion[3]);
        RCLCPP_INFO(get_logger(), "Client IP:%s\n", client_params.localAddress);
        RCLCPP_INFO(get_logger(), "Server IP:%s\n", client_params.serverAddress);
        RCLCPP_INFO(get_logger(), "Server Name:%s\n", server_description.szHostComputerName);

        void * pResult;
        int nBytes = 0;

        if (client->SendMessageAndWait("FrameRate", &pResult, &nBytes) == ErrorCode_OK) {
          float fRate = *(static_cast<float *>(pResult));
          RCLCPP_INFO(get_logger(), "Mocap Framerate : %3.2f\n", fRate);
        } else {
          RCLCPP_DEBUG(get_logger(), "Error getting frame rate.\n");
        }
      } else {
        RCLCPP_INFO(get_logger(), "... not connected :( ");
        return false;
      }

      return true;
    }


    bool disconnect_optitrack()
    {
        void * response;
        int nBytes;
        if (client->SendMessageAndWait("Disconnect", &response, &nBytes) == ErrorCode_OK) {
            client->Disconnect();
            RCLCPP_INFO(get_logger(), "[Client] Disconnected");
            return true;
        } else {
            RCLCPP_ERROR(get_logger(), "[Client] Disconnect not successful..");
            return false;
        }
    }


};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OptitrackDriverNode>();
    if (!node->connect_optitrack()) {
        RCLCPP_FATAL(node->get_logger(), "Could not connect to OptiTrack. Shutting down.");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}