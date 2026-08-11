cmake_minimum_required(VERSION 3.8)
project(navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/send_nav_goal.py scripts/run_success_trials.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
