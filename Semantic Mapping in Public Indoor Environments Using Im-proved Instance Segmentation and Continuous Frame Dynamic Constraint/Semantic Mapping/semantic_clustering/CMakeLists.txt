cmake_minimum_required(VERSION 3.8)
project(semantic_clustering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(semantic_clustering_node src/semantic_clustering_node.cpp)
ament_target_dependencies(semantic_clustering_node
  rclcpp
  visualization_msgs
  geometry_msgs
)

install(TARGETS
  semantic_clustering_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
