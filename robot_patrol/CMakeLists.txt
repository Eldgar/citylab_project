cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# **Find dependencies**
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# **Generate the custom service**
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
	"action/GoToPose.action"
	DEPENDENCIES geometry_msgs
	DEPENDENCIES nav_msgs
  DEPENDENCIES sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(sensor_msgs)

# **Add executables**
add_executable(patrol src/patrol.cpp)
ament_target_dependencies(patrol rclcpp sensor_msgs geometry_msgs)

add_executable(patrol_with_service src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service rclcpp sensor_msgs geometry_msgs)

add_executable(direction_service src/direction_service.cpp)
ament_target_dependencies(direction_service rclcpp sensor_msgs)

add_executable(go_to_pose_action src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_action rclcpp rclcpp_action sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)


add_executable(test_service src/test_service.cpp)
ament_target_dependencies(test_service rclcpp sensor_msgs)

add_executable(scan_index_value src/scan_index_value.cpp)
ament_target_dependencies(scan_index_value rclcpp sensor_msgs)

# **Link the generated interfaces to the executables**
rosidl_target_interfaces(patrol_with_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(direction_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(test_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(go_to_pose_action
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# **Install targets**
install(TARGETS
  patrol
  patrol_with_service
  go_to_pose_action
  direction_service
  test_service
  scan_index_value
  DESTINATION lib/${PROJECT_NAME})

# **Install launch files**
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

# Install RViz configuration files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()