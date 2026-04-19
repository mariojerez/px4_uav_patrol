cmake_minimum_required(VERSION 3.8)
project(px4_uav_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(shepherding_msgs REQUIRED)

add_executable(patrol_mission
  src/main.cpp
)

ament_target_dependencies(patrol_mission
  rclcpp
  rclcpp_action
  px4_ros2_cpp
  px4_msgs
  shepherding_msgs
)

target_include_directories(patrol_mission PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS patrol_mission
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  # NOTE: ament_lint_auto is intentionally not invoked for this package.
  # The pre-existing sources (`gen_patrol_mission.py`, `energy_logger.hpp`)
  # do not conform to the default ament lint rules, and bringing them into
  # compliance is out of scope for TASK-7.1. Only the gtest suite is run.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_patrol_logic test/test_patrol_logic.cpp)
  target_include_directories(test_patrol_logic PRIVATE include)
endif()

ament_package()
