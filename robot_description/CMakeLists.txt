cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

# Phase 1 will add: install(DIRECTORY urdf meshes launch DESTINATION share/${PROJECT_NAME})

ament_package()
