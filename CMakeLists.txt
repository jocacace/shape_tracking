cmake_minimum_required(VERSION 2.8.3)
project(shape_tracking)

add_compile_options(-std=c++11)
find_package(VISP REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  roscpp
  roslib
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES shape_tracking
#  CATKIN_DEPENDS cv_bridge image_transport roscpp roscpp roslib sensor_msgs std_msgs
#  DEPENDS system_lib
)

SET ( TRACKING_SRC
  tracking/ellipse_tracking.cpp
  tracking/sphere_stereo_tracking.cpp
)

include_directories(
  tracking
  ${catkin_INCLUDE_DIRS}
)

add_executable(shape_tracking src/shape_tracking.cpp src/camera_calibration.cpp ${TRACKING_SRC})
target_link_libraries(shape_tracking ${catkin_LIBRARIES} ${VISP_LIBRARIES})
