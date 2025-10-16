cmake_minimum_required(VERSION 3.5)
project(image_process)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 构建 ArmorMatcher 和 process 库
add_library(armor_detection SHARED
  include/ArmorMatcher.cpp
  include/process.cpp
)
target_link_libraries(armor_detection ${OpenCV_LIBS})

# 构建 ROS2 节点
add_executable(image_process_node src/image_process_node.cpp)
ament_target_dependencies(image_process_node rclcpp sensor_msgs cv_bridge std_msgs OpenCV)
target_link_libraries(image_process_node armor_detection ${OpenCV_LIBS})

install(TARGETS
  armor_detection
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  image_process_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
