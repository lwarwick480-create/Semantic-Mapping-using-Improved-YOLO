cmake_minimum_required(VERSION 3.8)
project(semantic_map_saver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Add semantic map saver node
add_executable(semantic_map_saver_node src/semantic_map_saver_node.cpp)
target_include_directories(semantic_map_saver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(semantic_map_saver_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(semantic_map_saver_node
  rclcpp
  nav_msgs
  visualization_msgs
  std_srvs
)
target_link_libraries(semantic_map_saver_node yaml-cpp)

# Add semantic map loader node
add_executable(semantic_map_loader_node src/semantic_map_loader_node.cpp)
target_include_directories(semantic_map_loader_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(semantic_map_loader_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(semantic_map_loader_node
  rclcpp
  nav_msgs
  visualization_msgs
  std_srvs
)
target_link_libraries(semantic_map_loader_node yaml-cpp)

# Install executables
install(TARGETS 
  semantic_map_saver_node
  semantic_map_loader_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
