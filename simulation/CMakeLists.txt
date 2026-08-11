cmake_minimum_required(VERSION 3.8)
project(simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/scan_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
