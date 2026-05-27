cmake_minimum_required(VERSION 3.8)
project(color_square_detector)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(detector_node src/detector_node.cpp)

ament_target_dependencies(detector_node
  rclcpp sensor_msgs std_msgs cv_bridge image_transport OpenCV)

install(TARGETS detector_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()