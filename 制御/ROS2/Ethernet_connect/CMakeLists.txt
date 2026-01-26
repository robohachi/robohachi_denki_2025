cmake_minimum_required(VERSION 3.8)
project(Ethernet_connect)

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

# 共通のソースとヘッダー
set(common_sources
  src/robohachi_udp.cpp
)

# 送信ノードの実行ファイル
add_executable(udp_send 
  src/send_udp.cpp 
  ${common_sources}
)
ament_target_dependencies(udp_send rclcpp sensor_msgs geometry_msgs)
target_include_directories(udp_send PRIVATE ./include)

# 自律送信ノードの実行ファイル
add_executable(udp_send_auto 
  src/send_udp_auto.cpp 
  ${common_sources}
)
ament_target_dependencies(udp_send_auto rclcpp geometry_msgs)
target_include_directories(udp_send_auto PRIVATE ./include)

# 受信ノードの実行ファイル
add_executable(udp_receive 
  src/receive_udp.cpp 
  ${common_sources}
)
ament_target_dependencies(udp_receive rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros)
target_include_directories(udp_receive PRIVATE ./include)

# インストール設定
install(TARGETS
  udp_send
  udp_send_auto
  udp_receive
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
