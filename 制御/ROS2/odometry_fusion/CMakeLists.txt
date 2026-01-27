cmake_minimum_required(VERSION 3.5)
project(odometry_fusion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(odom_fusion src/odom_fusion.cpp)
ament_target_dependencies(odom_fusion 
  rclcpp 
  nav_msgs 
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_include_directories(odom_fusion PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

install(TARGETS
  odom_fusion
  DESTINATION lib/${PROJECT_NAME})

ament_package()
