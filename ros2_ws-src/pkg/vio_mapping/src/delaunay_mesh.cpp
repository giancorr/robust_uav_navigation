#include "vio_mapping/delaunay_mesh.hpp"
#include <opencv2/opencv.hpp>
#include <unordered_map>

namespace smm {

SparseMesh DelaunayMesh::build(
    const LandmarkMap & landmarks,
    const CameraPose & camera_pose,
    const PinholeIntrinsics & K,
    CameraId camera) {

  SparseMesh mesh;

  // T_cam_world: inversa della posa camera, per portare i landmark
  // dal world frame al frame camera e poterli proiettare.
  Eigen::Isometry3d T_world_cam = Eigen::Isometry3d::Identity();
  T_world_cam.translate(camera_pose.position);
  T_world_cam.rotate(camera_pose.orientation);
  Eigen::Isometry3d T_cam_world = T_world_cam.inverse();

  // Mappa: indice del punto inserito in Subdiv2D -> id landmark globale,
  // così dopo la triangolazione possiamo recuperare le posizioni 3D world.
  std::vector<uint64_t> index_to_id;
  std::vector<cv::Point2f> pts2d;
  std::unordered_map<uint64_t, Eigen::Vector3d> id_to_world;

  pts2d.reserve(landmarks.size());
  index_to_id.reserve(landmarks.size());

  for (const auto & [id, lm] : landmarks) {
    bool visible_here =
        (camera == CameraId::FRONT) ? lm.seen_by_front : lm.seen_by_rear;
    if (!visible_here) continue;

    Eigen::Vector3d p_cam = T_cam_world * lm.position_world;
    if (p_cam.z() <= 0.05) continue; // dietro o troppo vicino al piano immagine

    double u = K.fx * (p_cam.x() / p_cam.z()) + K.cx;
    double v = K.fy * (p_cam.y() / p_cam.z()) + K.cy;
    if (u < 0 || v < 0 || u >= K.width || v >= K.height) continue;

    pts2d.emplace_back(static_cast<float>(u), static_cast<float>(v));
    index_to_id.push_back(id);
    id_to_world[id] = lm.position_world;
  }

  if (pts2d.size() < 3) {
    return mesh; // non abbastanza punti per triangolare
  }

  cv::Rect bounds(0, 0, K.width, K.height);
  cv::Subdiv2D subdiv(bounds);
  for (const auto & p : pts2d) subdiv.insert(p);

  std::vector<cv::Vec6f> triangle_list;
  subdiv.getTriangleList(triangle_list);

  // Per ogni triangolo 2D, ritroviamo i 3 punti originali (per coordinate
  // esatte, dato che Subdiv2D non restituisce direttamente gli indici)
  // e usiamo le loro posizioni world già note: questo è il "lifting" 3D.
  auto find_id_by_point = [&](const cv::Point2f & pt) -> int64_t {
    for (size_t i = 0; i < pts2d.size(); ++i) {
      if (std::abs(pts2d[i].x - pt.x) < 1e-2 && std::abs(pts2d[i].y - pt.y) < 1e-2) {
        return static_cast<int64_t>(index_to_id[i]);
      }
    }
    return -1;
  };

  for (const auto & t : triangle_list) {
    cv::Point2f p0(t[0], t[1]), p1(t[2], t[3]), p2(t[4], t[5]);
    // Scarta triangoli con vertici fuori dai bounds (artefatti Subdiv2D)
    if (!bounds.contains(p0) || !bounds.contains(p1) || !bounds.contains(p2)) continue;

    int64_t id0 = find_id_by_point(p0);
    int64_t id1 = find_id_by_point(p1);
    int64_t id2 = find_id_by_point(p2);
    if (id0 < 0 || id1 < 0 || id2 < 0) continue;

    MeshTriangle tri;
    tri.v0 = id_to_world[static_cast<uint64_t>(id0)];
    tri.v1 = id_to_world[static_cast<uint64_t>(id1)];
    tri.v2 = id_to_world[static_cast<uint64_t>(id2)];
    tri.source_camera = camera;
    mesh.triangles.push_back(tri);
  }

  return mesh;
}

}  // namespace smm
