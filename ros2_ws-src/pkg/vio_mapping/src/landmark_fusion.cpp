#include "vio_mapping/landmark_fusion.hpp"

namespace smm {

LandmarkFusion::LandmarkFusion(double merge_mahalanobis_threshold)
: merge_mahalanobis_threshold_(merge_mahalanobis_threshold) {}

uint64_t LandmarkFusion::allocateGlobalId() { return next_id_++; }

int64_t LandmarkFusion::findMergeCandidate(
    const Eigen::Vector3d & p_world,
    const Eigen::Matrix3d & cov_world) const {
  // Gating semplice: distanza di Mahalanobis rispetto alla covarianza somma.
  // Per dataset molto grandi questo loop va sostituito con un kd-tree
  // (es. nanoflann) indicizzato per posizione; lasciato O(N) per chiarezza.
  int64_t best_id = -1;
  double best_score = merge_mahalanobis_threshold_;

  for (const auto & [id, lm] : landmarks_) {
    Eigen::Vector3d diff = p_world - lm.position_world;
    Eigen::Matrix3d cov_sum = cov_world + lm.covariance;
    double score = std::sqrt(diff.transpose() * cov_sum.inverse() * diff);
    if (score < best_score) {
      best_score = score;
      best_id = static_cast<int64_t>(id);
    }
  }
  return best_id;
}

void LandmarkFusion::ingest(
    const std::vector<RawLandmarkObservation> & observations,
    const Eigen::Isometry3d & T_world_cam,
    CameraId camera) {
  for (const auto & obs : observations) {
    Eigen::Vector3d p_world = T_world_cam * obs.position_cam_or_imu_frame;

    // Propagazione covarianza nel world frame: Cov_world = R * Cov_cam * R^T
    Eigen::Matrix3d R = T_world_cam.rotation();
    Eigen::Matrix3d cov_world = R * obs.covariance * R.transpose();

    int64_t merge_id = findMergeCandidate(p_world, cov_world);

    if (merge_id >= 0) {
      // Fusione con landmark esistente (Step 6: multi-camera fusion)
      Landmark & lm = landmarks_.at(static_cast<uint64_t>(merge_id));

      // Fusione bayesiana semplice delle posizioni pesata per covarianza
      Eigen::Matrix3d K = lm.covariance * (lm.covariance + cov_world).inverse();
      lm.position_world = lm.position_world + K * (p_world - lm.position_world);
      lm.covariance = (Eigen::Matrix3d::Identity() - K) * lm.covariance;

      lm.last_observation = obs.stamp;
      lm.min_observation_distance =
          std::min(lm.min_observation_distance, obs.observation_distance);

      if (camera == CameraId::FRONT) {
        lm.seen_by_front = true;
        lm.id_front = obs.track_id;
      } else {
        lm.seen_by_rear = true;
        lm.id_rear = obs.track_id;
      }
    } else {
      // Nuovo landmark
      Landmark lm;
      lm.id = allocateGlobalId();
      lm.position_world = p_world;
      lm.covariance = cov_world;
      lm.originating_camera = camera;
      lm.last_observation = obs.stamp;
      lm.min_observation_distance = obs.observation_distance;
      if (camera == CameraId::FRONT) {
        lm.seen_by_front = true;
        lm.id_front = obs.track_id;
      } else {
        lm.seen_by_rear = true;
        lm.id_rear = obs.track_id;
      }
      landmarks_.emplace(lm.id, lm);
    }
  }
}

void LandmarkFusion::ingestWorldFrame(
    const std::vector<RawLandmarkObservation> & observations,
    CameraId camera) {
  for (const auto & obs : observations) {
    // I punti sono già nel world frame, niente trasformazione
    Eigen::Vector3d p_world = obs.position_cam_or_imu_frame;
    Eigen::Matrix3d cov_world = obs.covariance;

    int64_t merge_id = findMergeCandidate(p_world, cov_world);

    if (merge_id >= 0) {
      Landmark & lm = landmarks_.at(static_cast<uint64_t>(merge_id));
      Eigen::Matrix3d K = lm.covariance * (lm.covariance + cov_world).inverse();
      lm.position_world = lm.position_world + K * (p_world - lm.position_world);
      lm.covariance = (Eigen::Matrix3d::Identity() - K) * lm.covariance;
      lm.last_observation = obs.stamp;
      lm.min_observation_distance =
          std::min(lm.min_observation_distance, obs.observation_distance);
      if (camera == CameraId::FRONT) {
        lm.seen_by_front = true;
        lm.id_front = obs.track_id;
      } else {
        lm.seen_by_rear = true;
        lm.id_rear = obs.track_id;
      }
    } else {
      Landmark lm;
      lm.id = allocateGlobalId();
      lm.position_world = p_world;
      lm.covariance = cov_world;
      lm.originating_camera = camera;
      lm.last_observation = obs.stamp;
      lm.min_observation_distance = obs.observation_distance;
      if (camera == CameraId::FRONT) {
        lm.seen_by_front = true;
        lm.id_front = obs.track_id;
      } else {
        lm.seen_by_rear = true;
        lm.id_rear = obs.track_id;
      }
      landmarks_.emplace(lm.id, lm);
    }
  }
}

void LandmarkFusion::pruneStale(const rclcpp::Time & now, double max_age_seconds) {
  for (auto it = landmarks_.begin(); it != landmarks_.end();) {
    if (it->second.protected_flag) { ++it; continue; }
    double age = (now - it->second.last_observation).seconds();
    if (age > max_age_seconds) {
      it = landmarks_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace smm
