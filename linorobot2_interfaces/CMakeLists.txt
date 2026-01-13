cmake_minimum_required(VERSION 3.8)
project(linorobot2_interfaces)

find_package(ament_cmake REQUIRED)
# 1. We hebben de generators nodig
find_package(rosidl_default_generators REQUIRED)

# 2. Hier geven we aan welk bestand gecompileerd moet worden
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
