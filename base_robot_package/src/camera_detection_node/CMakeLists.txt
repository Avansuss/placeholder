cmake_minimum_required(VERSION 3.8)
project(camera_detection_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(camera_detection_node src/camera_detection_node.cpp)

ament_target_dependencies(camera_detection_node
  rclcpp
  std_msgs
  cv_bridge
  image_transport
)

target_link_libraries(camera_detection_node ${OpenCV_LIBRARIES})

install(TARGETS
  camera_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
