#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Core>
#include "utils.h"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <math.h>
//#include "TooN/TooN.h"
//#include "robohelper/robohelper.hpp"

#define OPTITRACK_BUF_SIZE 1000
#define OPTITRACK_PORT_DEF 9030

using namespace std;

using std::placeholders::_1;


struct optitrack_rigid_body_udp_packet {
	int ID;
	float x;
	float y;
	float z;
	float qw;
	float qx;
	float qy;
	float qz;
	int iFrame;
	float Latency;
	int nMarkers;
};
struct optitrack_other_markers_udp_packet {
	int iFrame;
	float Latency;
	int nMarkers;
};
struct optitrack_marker_udp_packet {
	int ID;
	float x;
	float y;
	float z;
};

/*typedef struct OPTITRACK_DATA {
	int ID;
	float x;
	float y;
	float z;
	float qw;
	float qx;
	float qy;
	float qz;
	int iFrame;
	float Latency;
} OPTITRACK_DATA;
*/

class OptitrackListener : public rclcpp::Node
{

  private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ot_odom_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    //params
    std::string odom_frame_id_;
    std::string map_frame_id_;
    bool publish_tf_;


    int opt_socket;
    std::stringstream intToString;
	  int rlen, slen;
	  unsigned char buffer[1024];
    sockaddr_in si_me, si_other;
	  optitrack_rigid_body_udp_packet optitrack_data;
    int port_;
	  bool blocking_;
	  bool debug_;
	  struct optitrack_rigid_body_udp_packet rb;
	  struct optitrack_other_markers_udp_packet oth_markers;
    string output_ref_frame_;

    std::thread run_thread_;


  public:
    OptitrackListener(): Node("px4_tf_pub"){

      //read params
      this->declare_parameter<string>("odom_frame_id", "ot_odom"); 
      odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();

      this->declare_parameter<string>("map_frame_id", "map"); 
      map_frame_id_ = this->get_parameter("map_frame_id").as_string();

      this->declare_parameter<bool>("publish_tf", true);
      publish_tf_ = this->get_parameter("publish_tf").as_bool();

      this->declare_parameter<int>("port", 9030);
      port_ = this->get_parameter("port").as_int();

      this->declare_parameter<bool>("debug", false);
      debug_ = this->get_parameter("debug").as_bool();

      this->declare_parameter<bool>("blocking", false);
      blocking_ = this->get_parameter("blocking").as_bool();

      this->declare_parameter<string>("output_ref_frame", "enu"); 
      output_ref_frame_ = this->get_parameter("output_ref_frame").as_string();


      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);


      ot_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/optitrack/odometry", qos);

      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


      run_thread_ = std::thread(&OptitrackListener::run, this);
      
    }

    ~OptitrackListener() {
      if (run_thread_.joinable()) {
        run_thread_.join();
      }
      //TODO: close socket
  }

    bool Open(int port_number){
      if ( (opt_socket=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        std::cout << "Listener::Open: error during socket creation!" << std::endl;
        return false;
      }
      else
        std::cout << "Listener::Open completed!" << std::endl;
    
      /* blocking or no-blocking */
      if (blocking_)
        fcntl(opt_socket,F_SETFL,O_NONBLOCK);
    
      memset((char *) &si_me, 0, sizeof(si_me));
    
      /* allow connections to any address port */
      si_me.sin_family = AF_INET;
      si_me.sin_port = htons(port_number);
      si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    
      if (bind(opt_socket, (struct sockaddr*)&si_me, sizeof(si_me))==-1) {
        std::cout << "Listener::Open: error during bind!" << std::endl;
        return false;
      }
      return true;
    }


    void Read(){

      // geometry_msgs::msg::PoseWithCovariance pose;
      nav_msgs::msg::Odometry ros_odom;
     
      slen=sizeof(si_other);
      rlen = recvfrom(opt_socket, buffer, 1024, 0,(struct sockaddr*)&si_other, (socklen_t*)&slen);
      char pck_type = (char)buffer[0];
    
      if (pck_type == 'O') { //Other markers
        oth_markers = *((struct optitrack_other_markers_udp_packet *)(buffer+1));
        if(debug_)
          printf("Other marker (iFrame %i, Latency %f ms, num_markers %i)\n", oth_markers.iFrame, oth_markers.Latency, oth_markers.nMarkers);
    
        int index = sizeof(struct optitrack_other_markers_udp_packet)+1;
        for (int j = 0; j<oth_markers.nMarkers; j++) {
          struct optitrack_marker_udp_packet marker;
          marker = *((struct optitrack_marker_udp_packet *)(buffer+index));
          index += sizeof(optitrack_marker_udp_packet);
          if (debug_)
            printf(" - marker #%i: %f, %f, %f m\n", marker.ID, marker.x, marker.y, marker.z);
        }
      }
      else if (pck_type == 'T') { //Trackable
        rb = *((struct optitrack_rigid_body_udp_packet *)(buffer+1));
    
    
        Eigen::Matrix3d R_o;
        R_o << 0, 0, 1, 
               1, 0, 0,
               0, 1, 0;
      
        //---Raw input
        Vector3d p_opt; // = Zeros;
        Vector4d q_opt; // = Zeros;
        p_opt << rb.x, rb.y, rb.z;
        q_opt << rb.qw, rb.qx, rb.qy, rb.qz;
        
        Matrix3d R_opt = utilities::QuatToMat( q_opt );
            
        // if(output_ref_frame_ == "ned"){
        //   Matrix3d R_o_ned;
        //   R_o_ned <<       0,  0, 1,
        //                   -1,  0, 0,
        //                   0, -1, 0;

        //   //Rotate optitrack data
        //   Vector3d p_opt2_ned = R_o_ned*p_opt;
        //   Matrix3d R_opt2_ned = R_o_ned*R_opt*R_o.transpose();
        //   Vector4d q_opt2_ned = utilities::rot2quat(R_opt2_ned);

        //   ros_odom.header.stamp = now();
        //   ros_odom.header.frame_id = odom_frame_id_;
        //   ros_odom.child_frame_id = map_frame_id_;   
        //   ros_odom.pose.pose.position.x = p_opt2_ned[0];
        //   ros_odom.pose.pose.position.y = p_opt2_ned[1];
        //   ros_odom.pose.pose.position.z = p_opt2_ned[2];
    
        //   ros_odom.pose.pose.orientation.w = q_opt2_ned[0];
        //   ros_odom.pose.pose.orientation.x = q_opt2_ned[1];
        //   ros_odom.pose.pose.orientation.y = q_opt2_ned[2];
        //   ros_odom.pose.pose.orientation.z = q_opt2_ned[3];  
        // }
        //else
         if(output_ref_frame_ == "enu"){
          Matrix3d R_o_enu;
          R_o_enu << -1,  0, 0,
                      0,  0, 1,
                      0,  1, 0;
          Vector3d p_opt2_enu = R_o_enu*p_opt;
          Matrix3d R_opt2_enu = R_o_enu*R_opt*R_o.transpose();
          Vector4d q_opt2_enu = utilities::rot2quat(R_opt2_enu);
    
          //cout << "R_enu: " << R_opt2_enu << endl;
          ros_odom.header.stamp = now();
          ros_odom.header.frame_id = map_frame_id_;
          ros_odom.child_frame_id = odom_frame_id_;   
          ros_odom.pose.pose.position.x = p_opt2_enu[0];
          ros_odom.pose.pose.position.y = p_opt2_enu[1];
          ros_odom.pose.pose.position.z = p_opt2_enu[2];
    
          ros_odom.pose.pose.orientation.w = q_opt2_enu[0];
          ros_odom.pose.pose.orientation.x = q_opt2_enu[1];
          ros_odom.pose.pose.orientation.y = q_opt2_enu[2];
          ros_odom.pose.pose.orientation.z = q_opt2_enu[3];  
        }
         
        
        ot_odom_pub_->publish(ros_odom);

        if(publish_tf_){
          geometry_msgs::msg::TransformStamped tf_odom;
          utilities::tf_from_odom(tf_odom,ros_odom);
          tf_broadcaster_->sendTransform(tf_odom);
        }
        
        
        int index = sizeof(struct optitrack_rigid_body_udp_packet)+1;
        for (int j = 0; j<rb.nMarkers; j++) {
          struct optitrack_marker_udp_packet marker;
          marker = *((struct optitrack_marker_udp_packet *)(buffer+index));
          index += sizeof(optitrack_marker_udp_packet);
          printf(" - marker #%i: %f, %f, %f m\n", marker.ID, marker.x, marker.y, marker.z);
        }
      }
    }

    
    
    int run(){

      rclcpp::Rate rate(150); //  Hz

      /* start listening socket */
      if ( !Open(port_) ) {
        cout << "Socket: opening failed!" << endl;
        return -1;
      }
      while(rclcpp::ok()) {
        Read();
        rate.sleep();
      }
      return 0;
    }
 


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OptitrackListener>());
  rclcpp::shutdown();
  return 0;
}