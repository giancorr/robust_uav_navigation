cmake_minimum_required(VERSION 3.8)
project(vio_health_monitor)

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

# ---------------------------------------------------------
# NODI ESISTENTI
# ---------------------------------------------------------

add_executable(degeneracy_monitor src/degeneracy_monitor.cpp)
ament_target_dependencies(degeneracy_monitor rclcpp std_msgs)

add_executable(emergency_rescue src/emergency_rescue.cpp)
ament_target_dependencies(emergency_rescue rclcpp std_msgs nav_msgs geometry_msgs px4_msgs)

add_executable(tactile_odometry src/tactile_odometry.cpp)
ament_target_dependencies(tactile_odometry rclcpp nav_msgs px4_msgs)

add_executable(surface_detector_node src/surface_detector_node.cpp)
ament_target_dependencies(surface_detector_node rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(surface_detector_node ${OpenCV_LIBRARIES})

add_executable(aruco_spawner src/aruco_spawner.cpp)
ament_target_dependencies(aruco_spawner rclcpp std_msgs nav_msgs geometry_msgs px4_msgs)

add_executable(aruco_spawner_down_node src/aruco_spawner_down_node.cpp)
ament_target_dependencies(aruco_spawner_down_node rclcpp std_msgs nav_msgs geometry_msgs px4_msgs)

add_executable(feature_counter_node src/feature_counter_node.cpp)
ament_target_dependencies(feature_counter_node rclcpp sensor_msgs std_msgs cv_bridge OpenCV)
target_link_libraries(feature_counter_node ${OpenCV_LIBRARIES})

add_executable(flight_data_logger src/flight_data_logger.cpp)
ament_target_dependencies(flight_data_logger rclcpp std_msgs nav_msgs px4_msgs)


# ---------------------------------------------------------
# NUOVI NODI (Wrench Estimator & Spray Heuristic)
# ---------------------------------------------------------

# Nodo Stimatore di Forza
add_executable(wrench_estimator_node src/external_wrench_estimator.cpp)
ament_target_dependencies(wrench_estimator_node 
  rclcpp 
  std_msgs 
  geometry_msgs 
  sensor_msgs 
  px4_msgs
)

# Nodo Euristica Target
add_executable(spray_heuristic_node src/spray_target_heuristic.cpp)
ament_target_dependencies(spray_heuristic_node 
  rclcpp 
  std_msgs 
  geometry_msgs
  px4_msgs
)

# ---------------------------------------------------------
# COLLEGAMENTO MESSAGGI CUSTOM (Typesupport)
# ---------------------------------------------------------

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(surface_detector_node ${cpp_typesupport_target})
target_link_libraries(emergency_rescue ${cpp_typesupport_target})
target_link_libraries(aruco_spawner ${cpp_typesupport_target})
target_link_libraries(aruco_spawner_down_node ${cpp_typesupport_target})
target_link_libraries(feature_counter_node ${cpp_typesupport_target})
# L'euristica quasi sicuramente leggerà SurfaceInfo o FeatureCount, quindi lo linkiamo
target_link_libraries(spray_heuristic_node ${cpp_typesupport_target})

# ---------------------------------------------------------
# INSTALLAZIONE
# ---------------------------------------------------------

install(TARGETS
  degeneracy_monitor
  emergency_rescue
  tactile_odometry
  surface_detector_node
  aruco_spawner
  aruco_spawner_down_node
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
