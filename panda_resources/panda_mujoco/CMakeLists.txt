cmake_minimum_required(VERSION 3.10.2)
project(panda_mujoco)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY franka_emika_panda DESTINATION share/${PROJECT_NAME})
