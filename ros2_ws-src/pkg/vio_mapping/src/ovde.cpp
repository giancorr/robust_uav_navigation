#include "vio_mapping/ovde.hpp"

namespace smm {

std::vector<VirtualPoint> OVDE::generate(
    const FreeSpacePolyhedron & poly,
    const Eigen::Vector3d & camera_center,
    CameraId camera,
    const std::vector<DirectionHistoryEntry> & direction_history) const {

  std::vector<VirtualPoint> virtual_points;
  virtual_points.reserve(direction_history.size());

  for (const auto & entry : direction_history) {
    // Condizione 1: parallasse storica sufficiente -> direzione "affidabile"
    if (entry.max_parallax_deg < params_.min_parallax_deg) continue;

    // Condizione 3: se in passato è SEMPRE stato rilevato un ostacolo in
    // quella direzione, non è "open area": meglio lasciare ai landmark reali.
    if (entry.ever_obstacle_detected) continue;

    // Condizione 2: nessun ostacolo conosciuto attualmente in quella
    // direzione fino a virtual_range (altrimenti il punto virtuale
    // sforerebbe dentro un ostacolo già mappato).
    double hit_dist;
    bool occluded = FreePolyhedronBuilder::isOccluded(
        poly, camera_center, entry.direction_world, params_.virtual_range, &hit_dist);
    if (occluded) continue;

    // Confidenza proporzionale alla parallasse osservata (più parallasse
    // storica = stima più affidabile), saturata a 1.0
    double confidence = std::min(1.0, entry.max_parallax_deg / (params_.min_parallax_deg * 3.0));
    if (confidence < params_.min_confidence) continue;

    VirtualPoint vp;
    vp.position_world = camera_center + entry.direction_world * params_.virtual_range;
    vp.source_camera = camera;
    vp.confidence = confidence;
    virtual_points.push_back(vp);
  }

  return virtual_points;
}

}  // namespace smm
