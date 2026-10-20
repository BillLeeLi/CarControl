cmake_minimum_required(VERSION 3.5)
project(process_node_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(PCL REQUIRED)
#find_package(laser_geometry REQUIRED)
find_package(robotsdk_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

#add_library(sync_store_lib
#  include/sync_store_node/sync_store_node.h
#  src/sync_store_node.cpp
#)
add_executable(process_node_cpp src/process_node_cpp.cpp)
ament_target_dependencies(process_node_cpp rclcpp std_msgs geometry_msgs sensor_msgs robotsdk_msgs vision_msgs OpenCV cv_bridge message_filters)
target_link_libraries(process_node_cpp ${OpenCV_LIBRARIES})

target_include_directories(process_node_cpp
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

install(TARGETS
        process_node_cpp
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


