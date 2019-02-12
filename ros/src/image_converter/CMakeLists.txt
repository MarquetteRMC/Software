cmake_minimum_required(VERSION 2.8.3)
project(image_converter)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    
  CONFIG
)



catkin_package()


include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


add_executable(image_converter_1 src/image_converter_node.cpp)
target_link_libraries(image_converter_1 ${catkin_LIBRARIES} ${OpenCV_LIBS})

