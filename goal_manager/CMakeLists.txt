cmake_minimum_required(VERSION 2.8.3)
project(goal_manager)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf message_generation genmsg actionlib_msgs actionlib)
#find_package(Boost REQUIRED COMPONENTS pending/disjoint_sets)

#add_action_files(DIRECTORY move_base_msgs FILES MoveBase.action)

add_message_files(
  FILES
  pose.msg
)

add_service_files(
  FILES
  position.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

#add_executable(cam_exploration src/cam_exploration.cpp)



#target_link_libraries(goal_manager ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)
