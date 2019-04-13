cmake_minimum_required(VERSION 2.8.3)
project(command_vel)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(my_command_vels src/command_vel.cpp)
target_link_libraries(my_command_vels ${catkin_LIBRARIES})

