cmake_minimum_required(VERSION 3.8)
project(antsy_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    mujoco
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/mujoco_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
