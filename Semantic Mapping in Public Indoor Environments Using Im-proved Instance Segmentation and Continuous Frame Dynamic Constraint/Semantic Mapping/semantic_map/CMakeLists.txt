cmake_minimum_required(VERSION 3.8)
project(semantic_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Create the executable
#add_executable(semantic_map_node src/semantic_map_node.cpp)
add_executable(semantic_map_node src/semantic_map_node_pair.cpp)
target_include_directories(semantic_map_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(semantic_map_node PUBLIC c_std_99 cxx_std_17)  # Require C++17

# Link against dependencies
ament_target_dependencies(semantic_map_node
  rclcpp
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_sensor_msgs
  geometry_msgs
  nav_msgs
)

# Install the executable
install(TARGETS semantic_map_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
