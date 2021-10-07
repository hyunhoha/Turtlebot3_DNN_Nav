cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# catkin_package(
#     INCLUDE_DIRS include
#     LIBRARIES
#     gazebo_ros_openni_kinect
# )

find_package(Boost REQUIRED COMPONENTS thread)

# Build our plugin

add_library(box_linear SHARED box_linear.cc)
target_link_libraries(box_linear ${GAZEBO_LIBRARIES})

add_library(box_plugin SHARED box_plugin.cc)
target_link_libraries(box_plugin ${GAZEBO_LIBRARIES})

add_library(obstacle_plugin SHARED obstacle_plugin.cc)
target_link_libraries(obstacle_plugin ${GAZEBO_LIBRARIES})

