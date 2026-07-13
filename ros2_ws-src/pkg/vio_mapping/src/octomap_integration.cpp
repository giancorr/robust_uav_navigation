#include "vio_mapping/octomap_integration.hpp"
#include <octomap_msgs/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace smm {

OctomapIntegration::OctomapIntegration(double resolution)
: tree_(resolution) {
  tree_.setProbHit(0.7);
  tree_.setProbMiss(0.4);
  tree_.setClampingThresMin(0.12);
  tree_.setClampingThresMax(0.97);
}

void OctomapIntegration::update(
    const std::vector<Eigen::Vector3d> & free_space_samples,
    const std::vector<Eigen::Vector3d> & occupied_points) {

  // I sample sono già garantiti "spazio libero osservato" per costruzione
  // (Step 3 + Step 4): aggiornarli come free è più cheap e più corretto del
  // raycasting standard, che assumerebbe densità di cui non disponiamo.
  for (const auto & p : free_space_samples) {
    tree_.updateNode(octomap::point3d(
        static_cast<float>(p.x()),
        static_cast<float>(p.y()),
        static_cast<float>(p.z())), false /* = free */);
  }

  // I landmark/vertici di mesh filtrati da DBOF sono i veri ostacoli.
  for (const auto & p : occupied_points) {
    tree_.updateNode(octomap::point3d(
        static_cast<float>(p.x()),
        static_cast<float>(p.y()),
        static_cast<float>(p.z())), true /* = occupied */);
  }

  tree_.updateInnerOccupancy();
}

sensor_msgs::msg::PointCloud2 OctomapIntegration::toDebugCloud(
    const std::string & frame_id) const {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  for (auto it = tree_.begin_leafs(); it != tree_.end_leafs(); ++it) {
    pcl::PointXYZRGB pt;
    pt.x = it.getX(); pt.y = it.getY(); pt.z = it.getZ();
    if (tree_.isNodeOccupied(*it)) {
      pt.r = 30; pt.g = 144; pt.b = 255;   // blu = occupied
    } else {
      pt.r = 30; pt.g = 200; pt.b = 60;   // verde = free
    }
    cloud.push_back(pt);
  }
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id;
  return msg;
}

octomap_msgs::msg::Octomap OctomapIntegration::toOctomapMsg(
    const std::string & frame_id) const {
  octomap_msgs::msg::Octomap msg;
  msg.header.frame_id = frame_id;
  octomap_msgs::binaryMapToMsg(tree_, msg);
  return msg;
}

}  // namespace smm
