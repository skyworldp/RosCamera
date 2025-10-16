cmake_minimum_required(VERSION 3.5)
project(hik_video_publisher_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(hik_video_publisher src/hik_video_publisher.cpp)
ament_target_dependencies(hik_video_publisher rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
  hik_video_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
