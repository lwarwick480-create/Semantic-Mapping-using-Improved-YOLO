cmake_minimum_required(VERSION 3.8)
project(yolo_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装 models 目录
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
