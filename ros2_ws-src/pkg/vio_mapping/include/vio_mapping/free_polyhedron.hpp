#pragma once

#include "vio_mapping/types.hpp"

namespace smm {

// -------------------------------------------------------------------------
// FreePolyhedronBuilder (Step 3)
//
// Costruisce il poliedro racchiuso tra il centro camera e la mesh di
// profondità sparsa: ogni triangolo della mesh, unito al centro camera,
// forma un "cono" (tetraedro degenere) che rappresenta spazio libero
// visibile garantito (nulla di osservato è più vicino del piano del
// triangolo, per costruzione vio_mapping).
//
// Front e rear vengono mantenuti come due fan di facce con apici diversi
// ma scritti nella stessa struttura, in modo che Step 6 (fusione) sia
// semplicemente "unione degli insiemi di facce".
// -------------------------------------------------------------------------
class FreePolyhedronBuilder {
public:
  static FreeSpacePolyhedron build(
      const SparseMesh & mesh_front,
      const SparseMesh & mesh_rear,
      const Eigen::Vector3d & camera_center_front,
      const Eigen::Vector3d & camera_center_rear);

  // Campiona N punti pseudo-random dentro il volume del poliedro (utile
  // per generare la point cloud sintetica finale, Step 7 opzione B).
  static std::vector<Eigen::Vector3d> sampleFreeSpace(
      const FreeSpacePolyhedron & poly,
      int samples_per_face = 5);

  // Restituisce true se il punto è "dentro" lo spazio libero osservato,
  // cioè più vicino alla camera del triangolo che lo occlude in quella
  // direzione. Usato da OVDE per evitare di generare punti virtuali oltre
  // un ostacolo già noto.
  static bool isOccluded(
      const FreeSpacePolyhedron & poly,
      const Eigen::Vector3d & ray_origin,
      const Eigen::Vector3d & ray_dir_normalized,
      double max_range,
      double * hit_distance_out = nullptr);
};

}  // namespace smm
