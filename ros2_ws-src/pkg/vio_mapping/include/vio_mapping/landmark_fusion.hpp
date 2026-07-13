#pragma once

#include "vio_mapping/types.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

// Messaggio landmark grezzo come arriva da OpenVINS, prima della fusione.
// Adatta i campi al tuo messaggio reale (es. openvins_msgs/Landmarks).
namespace smm {

struct RawLandmarkObservation {
  uint64_t track_id;                 // id del feature track in OpenVINS
  Eigen::Vector3d position_cam_or_imu_frame; // posizione nel frame nativo del msg
  Eigen::Matrix3d covariance;
  double observation_distance;       // distanza dal centro camera al momento dell'osservazione
  rclcpp::Time stamp;
};

// -------------------------------------------------------------------------
// LandmarkFusion (Step 1 + Step 6)
//
// Step 1: trasforma i landmark di ciascuna istanza OpenVINS nel world frame
//         usando le TF statiche conosciute (world->imu->cam) e la pose
//         IMU correntemente stimata.
// Step 6: landmark da front e rear convergono nella stessa mappa; se due
//         landmark di camere diverse cadono nella stessa "cella" di
//         distanza (gating mahalanobis), vengono fusi in un solo landmark
//         con seen_by_front=seen_by_rear=true, aumentando densità e FoV.
// -------------------------------------------------------------------------
class LandmarkFusion {
public:
  explicit LandmarkFusion(double merge_mahalanobis_threshold = 3.0);

  // Aggiorna/ingerisce le osservazioni di UNA camera in world frame.
  // T_world_cam: trasformazione corrente world<-cam per quella camera
  // (composizione di world->imu, nota da localizzazione, e imu->cam, TF statica).
  void ingest(
      const std::vector<RawLandmarkObservation> & observations,
      const Eigen::Isometry3d & T_world_cam,
      CameraId camera);

  // Variante per landmark già nel world frame (es. OpenVINS points_slam).
  // Salta la trasformazione T_world_cam.
  void ingestWorldFrame(
      const std::vector<RawLandmarkObservation> & observations,
      CameraId camera);

  // Rimuove landmark non osservati da più di max_age secondi (housekeeping,
  // ma NON sostituisce DBOF: qui si butta via solo roba davvero stantia).
  void pruneStale(const rclcpp::Time & now, double max_age_seconds);

  const LandmarkMap & landmarks() const { return landmarks_; }
  LandmarkMap & mutableLandmarks() { return landmarks_; }

private:
  uint64_t allocateGlobalId();
  // Cerca un landmark esistente compatibile (gating) per fondere l'osservazione.
  // Ritorna -1 se nessuna corrispondenza trovata.
  int64_t findMergeCandidate(
      const Eigen::Vector3d & p_world,
      const Eigen::Matrix3d & cov_world) const;

  LandmarkMap landmarks_;
  uint64_t next_id_ = 1;
  double merge_mahalanobis_threshold_;
};

}  // namespace smm
