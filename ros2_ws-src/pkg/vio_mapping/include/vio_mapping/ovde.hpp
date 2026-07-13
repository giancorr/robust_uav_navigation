#pragma once

#include "vio_mapping/types.hpp"
#include "vio_mapping/free_polyhedron.hpp"

namespace smm {

// -------------------------------------------------------------------------
// OVDE - Open Area Virtual Depth Estimation (Step 4)
//
// Per direzioni dove la mesh sparsa NON ha triangoli (buchi nella copertura,
// tipici con landmark sparsi), genera punti virtuali a distanza stimata,
// SOLO se:
//  - la direzione è stata osservata in passato (storico parallasse > soglia)
//  - non è attualmente occlusa da nessun triangolo noto (FreePolyhedron)
//  - la confidenza risultante supera min_confidence
//
// Questo enlarge il volume di spazio libero anche dove i landmark sparsi
// lasciano vuoti, evitando i "gap" che avresti facendo raycasting diretto
// sui soli landmark.
// -------------------------------------------------------------------------
struct OVDEParams {
  double min_parallax_deg = 5.0;      // parallasse minima storica richiesta
  double virtual_range = 6.0;         // profondità assegnata ai punti virtuali
  double angular_step_deg = 4.0;      // passo della griglia di direzioni testate
  double min_confidence = 0.4;
};

struct DirectionHistoryEntry {
  Eigen::Vector3d direction_world; // normalizzata
  double max_parallax_deg = 0.0;
  bool ever_obstacle_detected = false;
};

class OVDE {
public:
  explicit OVDE(const OVDEParams & params) : params_(params) {}

  // direction_history: storico per camera delle direzioni già osservate con
  // relativa parallasse massima accumulata (da aggiornare a monte, quando
  // arrivano le feature_tracks di OpenVINS).
  std::vector<VirtualPoint> generate(
      const FreeSpacePolyhedron & poly,
      const Eigen::Vector3d & camera_center,
      CameraId camera,
      const std::vector<DirectionHistoryEntry> & direction_history) const;

private:
  OVDEParams params_;
};

}  // namespace smm
