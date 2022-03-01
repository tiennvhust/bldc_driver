cmake_minimum_required(VERSION 3.0.2)
project(bldc_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  CppLinuxSerial
  vk_omni_plc
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bldc_driver
#  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(bldc_driver src/vk_motors.cpp src/BLDC.cpp)
target_link_libraries(bldc_driver ${catkin_LIBRARIES})

