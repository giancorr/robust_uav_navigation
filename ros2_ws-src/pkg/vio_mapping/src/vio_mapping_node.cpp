#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <octomap_msgs/msg/octomap.hpp>

#include "vio_mapping/types.hpp"
#include "vio_mapping/landmark_fusion.hpp"
#include "vio_mapping/delaunay_mesh.hpp"
#include "vio_mapping/free_polyhedron.hpp"
#include "vio_mapping/ovde.hpp"
#include "vio_mapping/dbof.hpp"
#include "vio_mapping/octomap_integration.hpp"

#include <functional>
#include <cmath>

using namespace std::chrono_literals;

namespace smm {

class VioMappingNode : public rclcpp::Node {
public:
  VioMappingNode()
  : Node("vio_mapping"),
    fusion_(/*merge_mahalanobis_threshold=*/3.0),
    octomap_(declare_parameter("octomap_resolution", 0.15)) {

    // --- Intrinseci baby_k (dalla calibrazione Kalibr) ---
    // cam0 (front) e cam1 (rear) hanno gli stessi intrinseci in simulazione
    intrinsics_front_.fx = declare_parameter("front.fx", 465.617);
    intrinsics_front_.fy = declare_parameter("front.fy", 465.617);
    intrinsics_front_.cx = declare_parameter("front.cx", 320.0);
    intrinsics_front_.cy = declare_parameter("front.cy", 240.0);
    intrinsics_front_.width = declare_parameter("front.width", 640);
    intrinsics_front_.height = declare_parameter("front.height", 480);

    intrinsics_rear_ = intrinsics_front_; // stesso modello in simulazione

    // --- Parametri pipeline ---
    ovde_params_.virtual_range = declare_parameter("ovde.virtual_range", 6.0);
    dbof_params_.redundancy_radius = declare_parameter("dbof.redundancy_radius", 0.15);
    dbof_params_.close_obstacle_threshold = declare_parameter("dbof.close_obstacle_threshold", 1.0);
    landmark_cov_scalar_ = declare_parameter("landmark_cov_scalar", 0.05);
    ovde_ = std::make_unique<OVDE>(ovde_params_);
    dbof_ = std::make_unique<DBOF>(dbof_params_);

    // --- T_imu_cam per cam0 (baby_k) ---
    T_imu_cam_front_ = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d R_ic;
    R_ic <<  0.0,  0.0,  1.0,
            -1.0,  0.0,  0.0,
             0.0, -1.0,  0.0;
    Eigen::Vector3d t_ic(0.21233, -0.03000, -0.01122);
    T_imu_cam_front_.linear() = R_ic;
    T_imu_cam_front_.translation() = t_ic;

    world_frame_ = declare_parameter("world_frame", std::string("global"));

    // --- Subscriber: singola istanza OpenVINS (fr_simulation.yml) ---
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/ov_msckf/odomimu", 10,
        std::bind(&VioMappingNode::onOdometry, this, std::placeholders::_1));

    landmarks_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ov_msckf/points_slam", 10,
        std::bind(&VioMappingNode::onLandmarks, this, std::placeholders::_1));

    // --- Publisher ---
    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
        "/octomap_binary", rclcpp::QoS(1).transient_local());
    debug_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/mapping/debug_cloud", 1);

    // Pipeline a 5 Hz
    timer_ = create_wall_timer(200ms, std::bind(&VioMappingNode::pipelineStep, this));

    RCLCPP_INFO(get_logger(), "vio_mapping avviato (sim mode, single OpenVINS).");
  }

private:
  // -----------------------------------------------------------------
  // Callback odometria: salva la posa corrente (T_world_imu) e calcola T_world_cam
  // -----------------------------------------------------------------
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    Eigen::Vector3d p_world_imu(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    Eigen::Quaterniond q_world_imu(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
    T_world_imu.translate(p_world_imu);
    T_world_imu.rotate(q_world_imu);

    // T_world_cam = T_world_imu * T_imu_cam
    Eigen::Isometry3d T_world_cam = T_world_imu * T_imu_cam_front_;

    CameraPose cp;
    cp.position = T_world_cam.translation();
    cp.orientation = Eigen::Quaterniond(T_world_cam.rotation());
    cp.id = CameraId::FRONT;
    cp.stamp = msg->header.stamp;
    pose_front_ = cp;
  }

  // -----------------------------------------------------------------
  // Callback landmark SLAM: parsa il PointCloud2 e ingerisce
  // -----------------------------------------------------------------
  void onLandmarks(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!pose_front_) return; // serve la posa per calcolare la distanza di osservazione

    std::vector<RawLandmarkObservation> observations;
    observations.reserve(msg->width * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    const double cov_val = landmark_cov_scalar_ * landmark_cov_scalar_;
    uint64_t synthetic_id = 0;

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      float x = *it_x, y = *it_y, z = *it_z;
      if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;

      Eigen::Vector3d p_world(x, y, z);
      double obs_dist = (p_world - pose_front_->position).norm();

      RawLandmarkObservation obs;
      obs.track_id = synthetic_id++;
      obs.position_cam_or_imu_frame = p_world; // già in world frame
      obs.covariance = Eigen::Matrix3d::Identity() * cov_val;
      obs.observation_distance = obs_dist;
      obs.stamp = msg->header.stamp;
      observations.push_back(obs);
    }

    if (!observations.empty()) {
      // Tutti i landmark sono trattati come "front" (unica istanza OpenVINS)
      fusion_.ingestWorldFrame(observations, CameraId::FRONT);
    }
  }

  // -----------------------------------------------------------------
  // Pipeline completa (Step 1→9)
  // -----------------------------------------------------------------
  void pipelineStep() {
    if (!pose_front_) return;

    // Housekeeping: rimuovi landmark stantii
    fusion_.pruneStale(now(), /*max_age_seconds=*/10.0);
    const LandmarkMap & landmarks = fusion_.landmarks();
    if (landmarks.empty()) {
        // Publish the (currently empty) octomap to ensure late subscribers unblock
        octomap_pub_->publish(octomap_.toOctomapMsg(world_frame_));
        return;
    }

    // Step 2: mesh sparsa (solo front, rear vuota)
    SparseMesh mesh_front = DelaunayMesh::build(
        landmarks, *pose_front_, intrinsics_front_, CameraId::FRONT);
    SparseMesh mesh_rear; // vuota: non abbiamo una posa rear separata

    // Step 3: poliedro di spazio libero
    FreeSpacePolyhedron poly = FreePolyhedronBuilder::build(
        mesh_front, mesh_rear, pose_front_->position, pose_front_->position);

    // Step 4: OVDE — disabilitato (direction_history vuoti)
    std::vector<DirectionHistoryEntry> direction_history_front;
    std::vector<DirectionHistoryEntry> direction_history_rear;

    std::vector<VirtualPoint> virtual_front =
        ovde_->generate(poly, pose_front_->position, CameraId::FRONT, direction_history_front);

    // Step 5: DBOF
    LandmarkMap filtered = landmarks;
    dbof_->filter(filtered);

    // Step 7: point cloud sintetica dal poliedro
    std::vector<Eigen::Vector3d> free_samples =
        FreePolyhedronBuilder::sampleFreeSpace(poly, /*samples_per_face=*/5);

    for (const auto & vp : virtual_front) free_samples.push_back(vp.position_world);

    std::vector<Eigen::Vector3d> occupied_points;
    occupied_points.reserve(filtered.size());
    for (const auto & [id, lm] : filtered) occupied_points.push_back(lm.position_world);

    // Step 8+9: aggiornamento Octomap
    octomap_.update(free_samples, occupied_points);

    // Pubblica
    debug_cloud_pub_->publish(octomap_.toDebugCloud(world_frame_));
    octomap_pub_->publish(octomap_.toOctomapMsg(world_frame_));
  }

  // --- stato ---
  LandmarkFusion fusion_;
  std::unique_ptr<OVDE> ovde_;
  std::unique_ptr<DBOF> dbof_;
  OctomapIntegration octomap_;

  OVDEParams ovde_params_;
  DBOFParams dbof_params_;
  PinholeIntrinsics intrinsics_front_, intrinsics_rear_;
  double landmark_cov_scalar_;

  std::optional<CameraPose> pose_front_;
  Eigen::Isometry3d T_imu_cam_front_;
  std::string world_frame_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr landmarks_sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace smm

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smm::VioMappingNode>());
  rclcpp::shutdown();
  return 0;
}
