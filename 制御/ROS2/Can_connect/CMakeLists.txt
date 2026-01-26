cmake_minimum_required(VERSION 3.8)
project(Can_connect)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(canable_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(can_send 
  src/can_send.cpp
)
ament_target_dependencies(can_send rclcpp sensor_msgs geometry_msgs canable_msgs)
target_include_directories(can_send PRIVATE ./include)

add_executable(can_rcv 
  src/can_receive.cpp
)
ament_target_dependencies(can_rcv rclcpp tf2_ros tf2 canable_msgs nav_msgs)
target_include_directories(can_rcv PRIVATE ./include)

add_executable(can_send_auto 
  src/can_send_auto.cpp
)
ament_target_dependencies(can_send_auto rclcpp sensor_msgs geometry_msgs canable_msgs)
target_include_directories(can_send_auto PRIVATE ./include)

# インストール設定
install(TARGETS
  can_send
  can_rcv
  can_send_auto
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
