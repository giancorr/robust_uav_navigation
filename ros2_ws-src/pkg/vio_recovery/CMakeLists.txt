cmake_minimum_required(VERSION 3.8)
project(vio_recovery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SurfaceInfo.msg"
  "msg/FeatureCount.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(degeneracy_monitor src/degeneracy_monitor.cpp)
ament_target_dependencies(degeneracy_monitor rclcpp std_msgs)

add_executable(vio_recovery_fsm src/vio_recovery_fsm.cpp)
ament_target_dependencies(vio_recovery_fsm rclcpp std_msgs nav_msgs geometry_msgs px4_msgs)

add_executable(vio_recovery_controller src/vio_recovery_controller.cpp)
ament_target_dependencies(vio_recovery_controller rclcpp std_msgs nav_msgs geometry_msgs px4_msgs)

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(flight_odometry_filter src/flight_odometry_filter.cpp)
ament_target_dependencies(flight_odometry_filter rclcpp nav_msgs px4_msgs geometry_msgs std_msgs tf2_ros tf2_geometry_msgs)



add_executable(swipe_spawner src/swipe_spawner.cpp)
ament_target_dependencies(swipe_spawner rclcpp std_msgs nav_msgs geometry_msgs px4_msgs)

add_executable(drop_spawner src/drop_spawner.cpp)
ament_target_dependencies(drop_spawner rclcpp std_msgs nav_msgs geometry_msgs px4_msgs)

add_executable(feature_counter_node src/feature_counter_node.cpp)
ament_target_dependencies(feature_counter_node rclcpp sensor_msgs std_msgs cv_bridge OpenCV)
target_link_libraries(feature_counter_node ${OpenCV_LIBRARIES})

add_executable(flight_data_logger src/flight_data_logger.cpp)
ament_target_dependencies(flight_data_logger rclcpp std_msgs nav_msgs px4_msgs)


add_executable(wrench_estimator_node src/external_wrench_estimator.cpp)
ament_target_dependencies(wrench_estimator_node 
  rclcpp 
  std_msgs 
  geometry_msgs 
  sensor_msgs 
  px4_msgs
)

add_executable(spray_heuristic_node src/spray_target_heuristic.cpp)
ament_target_dependencies(spray_heuristic_node 
  rclcpp 
  std_msgs 
  geometry_msgs
  px4_msgs
  nav_msgs
  sensor_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(vio_recovery_fsm ${cpp_typesupport_target})
target_link_libraries(swipe_spawner ${cpp_typesupport_target})
target_link_libraries(drop_spawner ${cpp_typesupport_target})
target_link_libraries(feature_counter_node ${cpp_typesupport_target})
target_link_libraries(spray_heuristic_node ${cpp_typesupport_target})

install(TARGETS
  degeneracy_monitor
  vio_recovery_fsm
  vio_recovery_controller
  flight_odometry_filter

  swipe_spawner
  drop_spawner
  feature_counter_node
  flight_data_logger
  wrench_estimator_node
  spray_heuristic_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
