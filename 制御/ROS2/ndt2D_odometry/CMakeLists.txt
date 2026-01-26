cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0144 NEW)
project(ndt2D_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ndt2d_odometry src/ndt2d_odometry.cpp)

ament_target_dependencies(ndt2d_odometry
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_conversions
)

target_link_libraries(ndt2d_odometry ${PCL_LIBRARIES})

install(TARGETS
  ndt2d_odometry
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
