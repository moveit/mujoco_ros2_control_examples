cmake_minimum_required(VERSION 3.5)
project(peg_in_hole)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)

set(THIS_PACKAGE_DEPENDS
  ament_cmake
  rclcpp
  moveit
  moveit_ros_planning_interface
  control_msgs
)

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller  ${THIS_PACKAGE_DEPENDS})

install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()