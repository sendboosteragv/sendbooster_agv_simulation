################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(sendbooster_agv_simulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

################################################################################
# Build
################################################################################
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

set(dependencies
  "geometry_msgs"
  "message_filters"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
)

set(EXEC_NAME "sendbooster_agv_drive")

add_executable(${EXEC_NAME} src/sendbooster_agv_drive.cpp)
ament_target_dependencies(${EXEC_NAME} ${dependencies})

# Laser Scan Merger
set(MERGER_NAME "laser_scan_merger")
add_executable(${MERGER_NAME} src/laser_scan_merger.cpp)
ament_target_dependencies(${MERGER_NAME} ${dependencies})

# Pose Reinitializer
set(REINIT_NAME "pose_reinitializer")
add_executable(${REINIT_NAME} src/pose_reinitializer.cpp)
ament_target_dependencies(${REINIT_NAME} ${dependencies})


################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME} ${MERGER_NAME} ${REINIT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models param rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/localization_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_package()
