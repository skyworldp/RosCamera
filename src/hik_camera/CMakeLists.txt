cmake_minimum_required(VERSION 3.8)

# Option to force use of clang/clang++ when available
option(USE_CLANG "Use clang/clang++ compilers" ON)

# If requested, try to locate clang and set it as the C/C++ compiler before
# the project() call so CMake configures languages with the selected compilers.
if(USE_CLANG)
  find_program(CLANG_PATH clang)
  find_program(CLANGXX_PATH clang++)
  if(CLANG_PATH AND CLANGXX_PATH)
    message(STATUS "USE_CLANG=ON: Found clang at ${CLANG_PATH} and clang++ at ${CLANGXX_PATH}; setting compilers to clang/clang++")
    set(CMAKE_C_COMPILER "${CLANG_PATH}" CACHE STRING "C compiler" FORCE)
    set(CMAKE_CXX_COMPILER "${CLANGXX_PATH}" CACHE STRING "C++ compiler" FORCE)
  else()
    message(STATUS "USE_CLANG=ON: clang or clang++ not found on PATH; leaving default compilers")
  endif()
endif()

project(hik_camera)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


add_executable(hik_camera_node src/hik_camera_node.cpp)
target_include_directories(hik_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/hik_camera>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hik_camera_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hik_camera_node
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "OpenCV"
)
## Compile HikCamera implementation (local source)
target_sources(hik_camera_node PRIVATE
  src/cameral_func/HikCamera.cpp
)

# Link against the Hikvision SDK libraries placed under include/hik_camera/lib/64
set(HIK_SDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/hik_camera/lib/64)
if(EXISTS ${HIK_SDK_LIB_DIR}/libMvCameraControl.so)
  message(STATUS "Found Hikvision SDK libs in ${HIK_SDK_LIB_DIR}")
  target_link_libraries(hik_camera_node ${HIK_SDK_LIB_DIR}/libMvCameraControl.so)
  # ensure runtime linker can find the .so when running from build tree
  set_target_properties(hik_camera_node PROPERTIES BUILD_RPATH "${HIK_SDK_LIB_DIR}" INSTALL_RPATH "${HIK_SDK_LIB_DIR}")
else()
  message(WARNING "Hikvision SDK library libMvCameraControl.so not found in ${HIK_SDK_LIB_DIR}. If you moved the libs elsewhere set HIK_SDK_LIB_DIR accordingly.")
endif()

install(TARGETS hik_camera_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files so ros2 launch can find them
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
