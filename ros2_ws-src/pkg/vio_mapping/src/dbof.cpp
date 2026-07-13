#include "vio_mapping/dbof.hpp"
#include <vector>
#include <algorithm>

namespace smm {

size_t DBOF::filter(LandmarkMap & landmarks) const {
  // Marca automaticamente come protetti gli ostacoli vicini noti, secondo
  // la regola 2: chi è stato visto vicino una volta resta "vicino" per
  // sempre nello storico, anche se ora osservato solo da lontano.
  for (auto & [id, lm] : landmarks) {
    if (lm.min_observation_distance < params_.close_obstacle_threshold) {
      lm.protected_flag = true;
    }
  }

  // Raggruppamento per ridondanza spaziale (regola 1): tra landmark molto
  // vicini tra loro, tieni solo quello con distanza di osservazione minima
  // (più affidabile), a meno che uno dei due sia protected (regola 3, e
  // di fatto anche regola 2 dato il marking sopra).
  std::vector<uint64_t> ids;
  ids.reserve(landmarks.size());
  for (auto & [id, lm] : landmarks) ids.push_back(id);

  std::vector<bool> to_remove(ids.size(), false);

  for (size_t i = 0; i < ids.size(); ++i) {
    if (to_remove[i]) continue;
    Landmark & li = landmarks.at(ids[i]);
    for (size_t j = i + 1; j < ids.size(); ++j) {
      if (to_remove[j]) continue;
      Landmark & lj = landmarks.at(ids[j]);

      double dist = (li.position_world - lj.position_world).norm();
      if (dist > params_.redundancy_radius) continue;

      // Mai rimuovere un landmark protetto, qualunque sia l'altro.
      if (li.protected_flag && lj.protected_flag) continue;
      if (li.protected_flag) { to_remove[j] = true; continue; }
      if (lj.protected_flag) { to_remove[i] = true; break; }

      // Nessuno dei due è protetto: tieni quello osservato più vicino.
      if (li.min_observation_distance <= lj.min_observation_distance) {
        to_remove[j] = true;
      } else {
        to_remove[i] = true;
        break; // li è stato eliminato, passa al prossimo i
      }
    }
  }

  size_t removed = 0;
  for (size_t i = 0; i < ids.size(); ++i) {
    if (to_remove[i]) {
      landmarks.erase(ids[i]);
      ++removed;
    }
  }
  return removed;
}

}  // namespace smm
