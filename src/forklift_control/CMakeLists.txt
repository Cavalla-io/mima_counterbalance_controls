cmake_minimum_required(VERSION 3.8)
project(forklift_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# JoyReader helper
add_library(joy_reader src/JoyReader.cpp)
target_include_directories(joy_reader PUBLIC include)
target_compile_features(joy_reader PUBLIC cxx_std_17)
ament_target_dependencies(joy_reader rclcpp sensor_msgs)

# DriveControl helper 
add_library(drive_logic src/DriveLogic.cpp)
target_include_directories(drive_logic PUBLIC include)
target_compile_features(drive_logic PUBLIC cxx_std_17)

add_library(safety_monitor src/SafetyMonitor.cpp)
target_include_directories(safety_monitor PUBLIC include)
target_compile_features(safety_monitor PUBLIC cxx_std_17)
ament_target_dependencies(safety_monitor rclcpp std_msgs sensor_msgs)

# SocketCAN adapter is header-only
add_library(socketcan_iface INTERFACE)
target_include_directories(socketcan_iface INTERFACE include)
target_compile_features(socketcan_iface INTERFACE cxx_std_17)

add_library(fork_logic src/ForkLogic.cpp)
target_include_directories(fork_logic PUBLIC include)
target_compile_features(fork_logic PUBLIC cxx_std_17)

# forklift_control_node
add_executable(ForkliftControlNode src/ForkliftControlNode.cpp)
target_include_directories(ForkliftControlNode PUBLIC include)
target_link_libraries(ForkliftControlNode joy_reader drive_logic fork_logic safety_monitor socketcan_iface)
ament_target_dependencies(ForkliftControlNode rclcpp sensor_msgs std_msgs)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  scripts/robot_status.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  joy_reader
  drive_logic
  safety_monitor
  fork_logic
  ForkliftControlNode
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
