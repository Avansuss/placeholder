cmake_minimum_required(VERSION 3.8)
project(robot_status_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(linorobot2_interfaces REQUIRED)

add_executable(robot_status_node src/robot_status_node.cpp)

ament_target_dependencies(robot_status_node
  rclcpp
  std_msgs
  linorobot2_interfaces
)

install(TARGETS
  robot_status_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
