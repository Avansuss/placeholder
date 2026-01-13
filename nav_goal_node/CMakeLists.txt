cmake_minimum_required(VERSION 3.8)
project(nav_goal_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(nav_goal_node src/nav_goal_node.cpp)

ament_target_dependencies(nav_goal_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  std_msgs
)

install(TARGETS
  nav_goal_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()



