cmake_minimum_required(VERSION 2.8.3)
project(locomotion_startup)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  fiducial_msgs
  geometry_msgs
  roscpp
  tf2_ros
  tf2_geometry_msgs
  tf
  tf2
  std_msgs
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(startup_locomotion src/startup_locomotion.cpp)
target_link_libraries(startup_locomotion ${catkin_LIBRARIES})

install(TARGETS startup_locomotion
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


##install(DIRECTORY launch/
##	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
##)

