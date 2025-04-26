cmake_minimum_required(VERSION 3.8)
project(path_planner_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rviz2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Install launch, config, and rviz files
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()