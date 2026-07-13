cmake_minimum_required(VERSION 3.8)
project(vio_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(vio_mapping_node
  src/vio_mapping_node.cpp
  src/landmark_fusion.cpp
  src/delaunay_mesh.cpp
  src/free_polyhedron.cpp
  src/ovde.cpp
  src/dbof.cpp
  src/octomap_integration.cpp
)

add_executable(vio_wall_densifier_node
  src/vio_wall_densifier_node.cpp
)

ament_target_dependencies(vio_mapping_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  octomap_msgs
)

ament_target_dependencies(vio_wall_densifier_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions

  pcl_ros
)

target_link_libraries(vio_mapping_node
  ${OCTOMAP_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS
  vio_mapping_node
  vio_wall_densifier_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
