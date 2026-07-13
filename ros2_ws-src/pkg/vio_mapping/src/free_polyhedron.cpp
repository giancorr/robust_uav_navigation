#include "vio_mapping/free_polyhedron.hpp"
#include <random>

namespace smm {

FreeSpacePolyhedron FreePolyhedronBuilder::build(
    const SparseMesh & mesh_front,
    const SparseMesh & mesh_rear,
    const Eigen::Vector3d & camera_center_front,
    const Eigen::Vector3d & camera_center_rear) {
  FreeSpacePolyhedron poly;
  poly.apex_front = camera_center_front;
  poly.apex_rear = camera_center_rear;
  poly.front_faces = mesh_front.triangles;
  poly.rear_faces = mesh_rear.triangles;
  return poly;
}

namespace {
// Interseca un raggio con un triangolo (Möller–Trumbore), ritorna true se
// c'è intersezione con t > 0 entro max_range.
bool rayTriangleIntersect(
    const Eigen::Vector3d & origin,
    const Eigen::Vector3d & dir,
    const MeshTriangle & tri,
    double max_range,
    double & t_out) {
  const double EPS = 1e-9;
  Eigen::Vector3d edge1 = tri.v1 - tri.v0;
  Eigen::Vector3d edge2 = tri.v2 - tri.v0;
  Eigen::Vector3d h = dir.cross(edge2);
  double a = edge1.dot(h);
  if (std::abs(a) < EPS) return false;
  double f = 1.0 / a;
  Eigen::Vector3d s = origin - tri.v0;
  double u = f * s.dot(h);
  if (u < 0.0 || u > 1.0) return false;
  Eigen::Vector3d q = s.cross(edge1);
  double v = f * dir.dot(q);
  if (v < 0.0 || u + v > 1.0) return false;
  double t = f * edge2.dot(q);
  if (t <= EPS || t > max_range) return false;
  t_out = t;
  return true;
}
}  // namespace

bool FreePolyhedronBuilder::isOccluded(
    const FreeSpacePolyhedron & poly,
    const Eigen::Vector3d & ray_origin,
    const Eigen::Vector3d & ray_dir_normalized,
    double max_range,
    double * hit_distance_out) {
  double best_t = max_range;
  bool hit = false;
  for (const auto & faces : {poly.front_faces, poly.rear_faces}) {
    for (const auto & tri : faces) {
      double t;
      if (rayTriangleIntersect(ray_origin, ray_dir_normalized, tri, max_range, t)) {
        if (t < best_t) { best_t = t; hit = true; }
      }
    }
  }
  if (hit && hit_distance_out) *hit_distance_out = best_t;
  return hit;
}

std::vector<Eigen::Vector3d> FreePolyhedronBuilder::sampleFreeSpace(
    const FreeSpacePolyhedron & poly,
    int samples_per_face) {
  std::vector<Eigen::Vector3d> out;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> uni(0.0, 1.0);

  auto sample_fan = [&](const std::vector<MeshTriangle> & faces,
                        const Eigen::Vector3d & apex) {
    for (const auto & tri : faces) {
      for (int s = 0; s < samples_per_face; ++s) {
        // Punto random sul triangolo (coordinate baricentriche)
        double r1 = uni(rng), r2 = uni(rng);
        if (r1 + r2 > 1.0) { r1 = 1.0 - r1; r2 = 1.0 - r2; }
        Eigen::Vector3d on_tri =
            tri.v0 + r1 * (tri.v1 - tri.v0) + r2 * (tri.v2 - tri.v0);
        // Punto random lungo il segmento apex -> on_tri: riempie il
        // volume del cono, non solo la superficie del triangolo.
        double t = uni(rng);
        out.push_back(apex + t * (on_tri - apex));
      }
    }
  };

  sample_fan(poly.front_faces, poly.apex_front);
  sample_fan(poly.rear_faces, poly.apex_rear);
  return out;
}

}  // namespace smm
