cmake_minimum_required(VERSION 3.5)
project(sync_store_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
add_executable(sync_store_node src/sync_store_node.cpp)
ament_target_dependencies(sync_store_node rclcpp std_msgs geometry_msgs sensor_msgs robotsdk_msgs vision_msgs OpenCV cv_bridge message_filters)
target_link_libraries(sync_store_node ${OpenCV_LIBRARIES})

target_include_directories(sync_store_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
        sync_store_node
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
