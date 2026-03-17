cmake_minimum_required(VERSION 3.8)
project(cloud_to_plane)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(cloud_to_plane_node src/cloud_to_plane_node.cpp)
target_include_directories(cloud_to_plane_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cloud_to_plane_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  cloud_to_plane_node
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS cloud_to_plane_node
  DESTINATION lib/${PROJECT_NAME})

ament_package() 
