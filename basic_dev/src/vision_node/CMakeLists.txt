cmake_minimum_required(VERSION 3.0.2)
project(vision_node)


set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4/)
find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  airsim_ros
  yolov11_ros_msgs
  roscpp
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  message_filters
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
)


add_executable(vision_test src/vision_test.cpp)
target_link_libraries(vision_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
