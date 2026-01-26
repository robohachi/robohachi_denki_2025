cmake_minimum_required(VERSION 3.8)
project(aruco_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(aruco_detector src/aruco_detector.cpp)
ament_target_dependencies(aruco_detector
    rclcpp sensor_msgs geometry_msgs cv_bridge tf2_ros OpenCV
)

install(TARGETS aruco_detector DESTINATION lib/${PROJECT_NAME})

ament_package()
