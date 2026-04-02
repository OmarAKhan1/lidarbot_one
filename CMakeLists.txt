cmake_minimum_required(VERSION 3.5)
project(lidarbot_one)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(lidarbot_one SHARED
  src/micro_ros_hardware.cpp
)

target_link_libraries(lidarbot_one PUBLIC
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
)

pluginlib_export_plugin_description_file(
  hardware_interface lidarbot_one_hardware.xml
)

install(TARGETS lidarbot_one
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/scan_restamper.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()