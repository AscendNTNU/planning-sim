# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(planning_ros_sim)

## Find catkin and any catkin packages + message_generation
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation)
## Declare ROS messages and services
#Må kanskje fjerne DIRECTORY msg før FILES??? ---------------------------------------------------

#add_service_files(DIRECTORY srv FILES)

add_message_files(FILES groundRobot.msg groundRobotList.msg droneCmd.msg)



## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime )




include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(perception_control src/perception_control.cpp)
target_link_libraries(perception_control ${catkin_LIBRARIES})
add_dependencies(perception_control planning_ros_sim_generate_messages_cpp)

add_executable(planning src/planning.cpp)
target_link_libraries(planning ${catkin_LIBRARIES})
add_dependencies(planning planning_ros_sim_generate_messages_cpp)


# %EndTag(FULLTEXT)%
