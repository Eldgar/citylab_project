cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add executable
add_executable(patrol src/patrol.cpp)
add_executable(scan_index_value src/scan_index_value.cpp)

ament_target_dependencies(scan_index_value rclcpp sensor_msgs)
# Link libraries
ament_target_dependencies(patrol rclcpp sensor_msgs geometry_msgs)

# Install executables
install(TARGETS patrol
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS scan_index_value
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
