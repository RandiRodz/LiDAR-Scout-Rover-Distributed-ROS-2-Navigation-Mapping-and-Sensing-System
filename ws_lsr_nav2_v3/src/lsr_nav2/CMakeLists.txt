cmake_minimum_required(VERSION 3.8)
project(point_cloud_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(advanced_proximity_guard src/advanced_proximity_guard.cpp)
ament_target_dependencies(advanced_proximity_guard rclcpp sensor_msgs pcl_conversions geometry_msgs visualization_msgs)
target_link_libraries(advanced_proximity_guard ${PCL_LIBRARIES})

add_executable(nav2_obstacle_cloud src/nav2_obstacle_cloud.cpp)
ament_target_dependencies(nav2_obstacle_cloud rclcpp sensor_msgs pcl_conversions visualization_msgs)
target_link_libraries(nav2_obstacle_cloud ${PCL_LIBRARIES})

add_executable(lidar_retimestamp_bridge src/lidar_retimestamp_bridge.cpp)
ament_target_dependencies(lidar_retimestamp_bridge rclcpp sensor_msgs)

add_executable(cmd_vel_mux src/cmd_vel_mux.cpp)
ament_target_dependencies(cmd_vel_mux rclcpp geometry_msgs)

add_executable(odom_stabilizer src/odom_stabilizer.cpp)
ament_target_dependencies(odom_stabilizer rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

add_executable(odom_retimestamp_bridge src/odom_retimestamp_bridge.cpp)
ament_target_dependencies(odom_retimestamp_bridge rclcpp nav_msgs tf2 tf2_msgs tf2_ros)

install(TARGETS
  advanced_proximity_guard nav2_obstacle_cloud lidar_retimestamp_bridge cmd_vel_mux odom_stabilizer odom_retimestamp_bridge
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/gas_data_processor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
