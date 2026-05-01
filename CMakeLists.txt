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

# Calibration suite: pure-Python mission generator, runner, fitter, and SRS
# updater. Installed under share/ so launch files and the runner can locate
# them via ament_index_python.get_package_share_directory().
install(DIRECTORY calibration/
  DESTINATION share/${PROJECT_NAME}/calibration
  FILES_MATCHING PATTERN "*.py"
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

if(BUILD_TESTING)
  # NOTE: ament_lint_auto is intentionally not invoked for this package.
  # The pre-existing sources (`gen_patrol_mission.py`, `energy_logger.hpp`)
  # do not conform to the default ament lint rules, and bringing them into
  # compliance is out of scope for TASK-7.1. Only the gtest suite is run.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_patrol_logic test/test_patrol_logic.cpp)
  target_include_directories(test_patrol_logic PRIVATE include)

  # EnergyLogger snapshot-mode tests need rclcpp + px4_msgs (for the message
  # types in the synthetic injections) + px4_ros2_cpp (for the bundled
  # nlohmann/json header used by the meta JSON parser).
  ament_add_gtest(test_energy_logger_snapshot
    test/test_energy_logger_snapshot.cpp
    ENV "CYCLONEDDS_URI=")
  target_include_directories(test_energy_logger_snapshot PRIVATE include)
  ament_target_dependencies(test_energy_logger_snapshot
    rclcpp
    px4_msgs
    px4_ros2_cpp
  )

  # Python tests for the calibration suite. Reuses pytest, no rclpy needed.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gen_calibration_missions
    test/test_gen_calibration_missions.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_fit_energy_constants
    test/test_fit_energy_constants.py
    TIMEOUT 120
  )
  ament_add_pytest_test(test_update_srs
    test/test_update_srs.py
    TIMEOUT 30
  )
endif()

ament_package()
