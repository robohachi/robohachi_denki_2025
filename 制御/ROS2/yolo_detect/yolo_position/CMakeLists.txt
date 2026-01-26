cmake_minimum_required(VERSION 3.8)
project(yolo_position)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(yolo_position_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(yolo_position src/yolo_position.cpp)
target_include_directories(yolo_position PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(yolo_position rclcpp yolo_msgs yolo_position_msgs geometry_msgs)

install(TARGETS yolo_position DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
