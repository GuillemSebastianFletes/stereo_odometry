cmake_minimum_required(VERSION 2.8.3)
project(stereo_odometry)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package( OpenCV REQUIRED )

catkin_package()

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


## Declare a cpp library
 add_library(stereo_odometry src/stereo_odometry.cpp)
 target_link_libraries(stereo_odometry
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${BOOST_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

## Declare a cpp executable
add_executable(stereo_odometry_node src/stereo_odometry_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(stereo_odometry_node stereo_odometry_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(stereo_odometry_node
   stereo_odometry
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${BOOST_LIBRARIES}
   ${PCL_COMMON_LIBRARIES}
   ${PCL_IO_LIBRARIES}
 )


