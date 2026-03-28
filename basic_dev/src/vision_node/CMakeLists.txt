cmake_minimum_required(VERSION 3.0.2)
project(vision_node)


set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4/)
find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET)

find_package(catkin REQUIRED COMPONENTS
  airsim_ros
  yolov11_ros_msgs
  roscpp
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  message_filters
  pcl_ros
  path_sender
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control
#  CATKIN_DEPENDS airsim_ros roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(vision_test src/vision_test.cpp)
target_link_libraries(vision_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(main src/vision_node.cpp)
target_include_directories(main PRIVATE include/)
target_link_libraries(main 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(lidar_converter_node src/liadr_transfer.cpp)
target_link_libraries(lidar_converter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARY_DIRS}
)

	