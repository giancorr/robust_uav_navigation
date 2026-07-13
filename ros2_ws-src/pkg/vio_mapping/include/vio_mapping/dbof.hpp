#pragma once

#include "vio_mapping/types.hpp"

namespace smm {

// -------------------------------------------------------------------------
// DBOF - Distance-Based Obstacle Filtering (Step 5)
//
// Regole richieste:
//  1) Tra landmark "duplicati"/ridondanti in una stessa zona, preferisci
//     quello osservato a distanza più corta (più affidabile per evitare
//     ostacoli vicini).
//  2) NON cancellare un ostacolo vicino anche se ultimamente è osservato
//     solo da lontano (lo storico min_observation_distance vince sempre
//     sull'osservazione corrente: un muro visto una volta a 0.5 m resta
//     "vicino" anche se ora lo vedi a 4 m).
//  3) Mantenere i landmark marcati come protected_flag indipendentemente
//     da età o ridondanza (es. ostacoli critici inseriti manualmente o
//     confermati da più sensori).
// -------------------------------------------------------------------------
struct DBOFParams {
  double redundancy_radius = 0.15;     // raggio per considerare due landmark "duplicati"
  double close_obstacle_threshold = 1.0; // sotto questa distanza un landmark è "ostacolo vicino"
};

class DBOF {
public:
  explicit DBOF(const DBOFParams & params) : params_(params) {}

  // Filtra in place la mappa landmark, applicando le 3 regole sopra.
  // Ritorna il numero di landmark rimossi (solo a scopo di logging).
  size_t filter(LandmarkMap & landmarks) const;

private:
  DBOFParams params_;
};

}  // namespace smm
