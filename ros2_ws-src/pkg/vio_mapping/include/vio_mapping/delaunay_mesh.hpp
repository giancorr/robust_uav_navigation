#pragma once

#include "vio_mapping/types.hpp"

namespace smm {

// Modello pinhole minimo necessario per proiettare i landmark nell'immagine
// di ciascuna camera. Riempi con i parametri di calibrazione del T265
// (fx, fy, cx, cy) per il fisheye rettificato, o usa il tuo modello
// equidistant/kannala-brandt se preferisci proiettare senza rettifica.
struct PinholeIntrinsics {
  double fx, fy, cx, cy;
  int width, height;
};

// -------------------------------------------------------------------------
// DelaunayMesh (Step 2)
//
// 1) Proietta i landmark visibili (quelli con originating/seen_by ==
//    questa camera, e davanti al piano immagine) nelle coordinate pixel.
// 2) Calcola la triangolazione di Delaunay 2D (cv::Subdiv2D).
// 3) "Lifta" ogni triangolo 2D nello spazio 3D usando le posizioni world
//    già note dei landmark corrispondenti: il risultato è una mesh di
//    profondità sparsa, esattamente come in vio_mapping.
// -------------------------------------------------------------------------
class DelaunayMesh {
public:
  // camera_pose: posa corrente world<-cam
  // landmarks: tutti i landmark fusi; vengono filtrati internamente per
  //            quelli visibili da questa camera.
  static SparseMesh build(
      const LandmarkMap & landmarks,
      const CameraPose & camera_pose,
      const PinholeIntrinsics & intrinsics,
      CameraId camera);
};

}  // namespace smm
