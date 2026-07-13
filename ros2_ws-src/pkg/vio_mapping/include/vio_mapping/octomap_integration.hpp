#pragma once

#include "vio_mapping/types.hpp"
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace smm {

// -------------------------------------------------------------------------
// OctomapIntegration (Step 7, opzione B)
//
// 1) Inserisce nell'OcTree, per ogni camera, raggi origin->free_sample come
//    "free" (insertRay normale, oppure setNodeValue diretto per i punti
//    campionati dal poliedro, che sono GIA' garantiti free per costruzione,
//    quindi non serve nemmeno il raycasting completo: basta marcare le
//    celle occupate dai sample come free, ed è molto più efficiente).
// 2) Inserisce i landmark (e i virtual point OVDE a bassa confidenza NON
//    vengono trattati come occupied, solo i landmark reali lo sono) come
//    "occupied" nell'OcTree, come endpoint dei triangoli della mesh.
// 3) Esporta sia un sensor_msgs/PointCloud2 di debug sia il messaggio
//    octomap_msgs/Octomap compresso, pronto per RViz e per il planner.
// -------------------------------------------------------------------------
class OctomapIntegration {
public:
  explicit OctomapIntegration(double resolution = 0.15);

  // free_space_samples: punti dentro il poliedro (Step 3), marcati free.
  // occupied_points: vertici della mesh / landmark filtrati da DBOF (Step5),
  //                   marcati occupied.
  void update(
      const std::vector<Eigen::Vector3d> & free_space_samples,
      const std::vector<Eigen::Vector3d> & occupied_points);

  sensor_msgs::msg::PointCloud2 toDebugCloud(const std::string & frame_id) const;
  octomap_msgs::msg::Octomap toOctomapMsg(const std::string & frame_id) const;

  octomap::OcTree & tree() { return tree_; }

private:
  octomap::OcTree tree_;
};

}  // namespace smm
