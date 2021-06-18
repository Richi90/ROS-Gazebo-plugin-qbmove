cmake_minimum_required(VERSION 3.0.2)
project(command_signal)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES command_signal
 CATKIN_DEPENDS gazebo-ros roscpp rospy std_msgs
 DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(give_sin_node src/give_sin.cpp)

target_link_libraries(give_sin_node
  ${catkin_LIBRARIES}
)