# VIO Mapping

> **Acknowledgement**: This package is heavily inspired by the original work [MonoSpheres by ctu-mrs](https://github.com/ctu-mrs/monospheres/tree/master), as described in their paper *"MonoSpheres: Large-Scale Monocular SLAM-Based UAV Exploration Through Perception-Coupled Mapping and Planning"*. 

ROS 2 / C++ implementation of the mapping pipeline: MonoSpheres-style sparse mesh + free space polyhedron + OVDE + DBOF, fed by two OpenVINS instances (front/rear), with direct output as an Octomap.

## Differences from the original work
While the core algorithmic concept is inspired by MonoSpheres, the code in this repository has been rewritten from scratch to fit our specific requirements:

1. **ROS 2 & Pure C++17:** The original project uses ROS 1 and heavily relies on Python and Open3D for mesh management. This implementation is 100% native C++ on ROS 2, eliminating the Python/Open3D bottleneck and ensuring hard-real-time performance at very high frequencies.
2. **Native Multi-Camera Support:** Instead of being limited to a single camera, the architecture was designed from the ground up to handle two simultaneous instances of OpenVINS (front and rear). Landmarks from both cameras are natively fused into a single global space (Steps 1 and 3).
3. **Direct Octomap Output:** Unlike the original implementation, our node internally converts the free space polyhedron and the landmarks, directly publishing a standard `octomap_msgs/Octomap` message ready to be used by the path planner.
4. **Independence from the MRS System:** The original code is tightly coupled with the MRS UAV system. This version is completely agnostic: it consumes standard ROS 2 messages (Odometry, PointCloud2), making it plug-and-play with PX4 and our custom `babyk_drone_manager` stack.

---

## Pipeline Step -> File Mapping

| Step | What it does | File |
|---|---|---|
| 1) Receive landmarks | transforms to world frame, fuses duplicates | `landmark_fusion.{hpp,cpp}` |
| 2) Build Sparse Visible Mesh | projection + 2D Delaunay + 3D lifting | `delaunay_mesh.{hpp,cpp}` |
| 3) Build Visible Free Space Polyhedron | camera->triangle cones, ray intersection | `free_polyhedron.{hpp,cpp}` |
| 4) Open Area Virtual Depth (OVDE) | virtual points in coverage holes | `ovde.{hpp,cpp}` |
| 5) Landmark Filtering (DBOF) | distance/protection rules | `dbof.{hpp,cpp}` |
| 6) Multi-Camera Fusion | implicit: front+rear fused in Steps 1 and 3 | `landmark_fusion.cpp` (ingest per camera) + `free_polyhedron.cpp` (front_faces+rear_faces) |
| 7) Output | polyhedron sampling -> point cloud -> Octomap | `octomap_integration.{hpp,cpp}` |
| ROS 2 Node | orchestration, subscriber/publisher | `vio_mapping_node.cpp` |

## What needs to be adapted before compiling

1. **Real landmark message type.** I left a commented placeholder in the node (`onLandmarks`, `openvins_msgs::msg::Landmarks`): replace it with the actual type published by your OpenVINS build (check whether it publishes per-landmark covariance or just position+id).
2. **imu->cam TF composition.** The node assumes that `/openvins_*/pose` directly publishes the pose you want to use as the camera pose. If it publishes the IMU pose instead, you must compute `T_world_cam = T_world_imu * T_imu_cam` using the known static TF (`Timu_cam` from your document) before passing the pose to `DelaunayMesh::build` and `fusion_.ingest(...)`.
3. **Camera intrinsics.** `intrinsics_front_`/`intrinsics_rear_` in `vio_mapping_node.cpp` are placeholders: they must be replaced with the real calibration of the T265 fisheye (if working on pinhole rectified images), or the projector must be replaced with an equidistant/Kannala-Brandt model if projecting directly on the raw fisheye.
4. **Merge performance in `LandmarkFusion::findMergeCandidate`.** It is O(N) for each ingested landmark: if you have many hundreds of landmarks per frame, replace it with a kd-tree (e.g. nanoflann) or a spatial hash grid.
5. **Subdiv2D in `DelaunayMesh::build`.** OpenCV `Subdiv2D` does not directly return the indices of inserted points, so the code matches them by coordinates (`find_id_by_point`, O(N) per triangle). This is fine for tens/hundreds of landmarks per camera; if you exceed a few hundred per frame, pre-build a `std::unordered_map<pair<int,int>, id>` with quantized coordinates.

## Build

```bash
cd ~/ros2_ws/src
# copy/clone the vio_mapping folder here
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select vio_mapping
source install/setup.bash
ros2 run vio_mapping vio_mapping_node
```

System dependencies in addition to standard ROS 2 ones: `libopencv-dev`, `libpcl-dev` (usually already present if you use RTAB-Map), `liboctomap-dev`.

## Notes on integration with your Octomap/RTAB-Map based planner

- The node publishes both `/octomap` (`octomap_msgs/Octomap`, compressed) and `/mapping/debug_cloud` (colorized free/occupied PointCloud2) for RViz.
- Since we generate the "synthetic" point cloud ourselves here, **there is no need** to go through the standard `octomap_server` node with real depth raycasting: insertion is already done with correct semantics (free from samples in the polyhedron, occupied from DBOF-filtered landmarks). If your planner still expects the OcTree from `octomap_server`, you can still feed it the debug point cloud as if it were a synthetic "depth cloud", but it is redundant: it's better to consume `/octomap` directly from here.
- For RTAB-Map: if you only use it for the pose/loop closure graph and not for occupancy, this node can work in parallel by taking poses from OpenVINS (as it does now) without any dependency on RTAB-Map. If you instead want RTAB-Map itself to consume this map, the simplest hook point is to publish the debug point cloud on the topic that RTAB-Map uses as input "scan"/"cloud" for its 2D/3D occupancy grid.
