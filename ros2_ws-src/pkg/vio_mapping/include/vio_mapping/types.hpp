#pragma once

#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <memory>
#include <rclcpp/time.hpp>

namespace smm {

enum class CameraId { FRONT = 0, REAR = 1 };

// -------------------------------------------------------------------------
// Landmark fuso nel frame world, come richiesto nello Step 1.
// -------------------------------------------------------------------------
struct Landmark {
  uint64_t id;                       // id univoco globale (dopo merge)
  uint64_t id_front = 0;              // id originale lato OpenVINS Front (0 = non visto)
  uint64_t id_rear  = 0;              // id originale lato OpenVINS Rear

  Eigen::Vector3d position_world;     // posizione in world frame
  Eigen::Matrix3d covariance;         // covarianza 3x3 in world frame

  CameraId originating_camera;        // camera che lo ha osservato per prima
  bool seen_by_front = false;
  bool seen_by_rear  = false;

  rclcpp::Time last_observation;      // timestamp ultima osservazione
  double min_observation_distance = std::numeric_limits<double>::max();

  bool protected_flag = false;        // landmark protetto da DBOF (Step 5)
};

using LandmarkMap = std::unordered_map<uint64_t, Landmark>;

// -------------------------------------------------------------------------
// Posa di una camera nel mondo (da OpenVINS, dopo TF statica imu->cam)
// -------------------------------------------------------------------------
struct CameraPose {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  CameraId id;
  rclcpp::Time stamp;
};

// -------------------------------------------------------------------------
// Triangolo 3D della mesh sparsa (Step 2)
// -------------------------------------------------------------------------
struct MeshTriangle {
  Eigen::Vector3d v0, v1, v2;     // vertici in world frame
  CameraId source_camera;         // da quale camera è stato generato
};

struct SparseMesh {
  std::vector<MeshTriangle> triangles;
};

// -------------------------------------------------------------------------
// Poliedro di spazio libero visibile (Step 3): rappresentato come insieme
// di "coni" camera_center -> triangolo, comodo per campionamento e per OVDE.
// -------------------------------------------------------------------------
struct FreeSpacePolyhedron {
  Eigen::Vector3d apex_front;   // camera center front
  Eigen::Vector3d apex_rear;    // camera center rear
  std::vector<MeshTriangle> front_faces;
  std::vector<MeshTriangle> rear_faces;
};

// -------------------------------------------------------------------------
// Punto virtuale generato da OVDE (Step 4)
// -------------------------------------------------------------------------
struct VirtualPoint {
  Eigen::Vector3d position_world;
  CameraId source_camera;
  double confidence; // 0..1, basata su parallasse e persistenza
};

}  // namespace smm
