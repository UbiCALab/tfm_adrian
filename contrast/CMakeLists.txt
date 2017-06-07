cmake_minimum_required(VERSION 2.4.6)
project(contrast_augmenter)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg sensor_msgs cv_bridge image_transport)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(contrast_augmenter src/contrast_augmenter.cpp)
target_link_libraries(contrast_augmenter ${catkin_LIBRARIES})
#add_dependencies(contrast_augmenter contrast_augmenter_generate_messages_cpp)

