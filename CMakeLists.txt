# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(planning_ros_sim)

## Find catkin and any catkin packages + message_generation
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation actionlib
	actionlib_msgs ascend_msgs)
## Declare ROS messages and services

#add_service_files(DIRECTORY srv FILES)

add_message_files(FILES groundRobot.msg groundRobotList.msg droneCmd.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(
	CATKIN_DEPENDS message_runtime roscpp ascend_msgs
	LIBRARIES libAI
	roscpp ascend_msgs
)

add_library(libAI src/AI/World.cpp src/AI/Plank.cpp src/AI/Robot.cpp src/AI/Drone.cpp src/AI/Observation.cpp src/AI/AI.cpp src/AI/AIController.cpp)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
include_directories(include ${catkin_INCLUDE_DIRS})

if(NOT ($ENV{ASCEND_ENV} MATCHES "^production$"))
	add_executable(perception_control src/perception_control.cpp)
	target_link_libraries(perception_control ${catkin_LIBRARIES})
	add_dependencies(perception_control planning_ros_sim_generate_messages_cpp)

	add_executable(action_safety_checker src/action_safety_checker.cpp)
	target_link_libraries(action_safety_checker ${catkin_LIBRARIES})
	add_dependencies(action_safety_checker planning_ros_sim_generate_messages_cpp)

	add_executable(benchmark_perception_control src/benchmark_perception_control.cpp)
	target_link_libraries(benchmark_perception_control ${catkin_LIBRARIES})
	add_dependencies(benchmark_perception_control planning_ros_sim_generate_messages_cpp)
endif()

add_executable(planning src/planning.cpp)
target_link_libraries(planning libAI ${catkin_LIBRARIES})
add_dependencies(planning planning_ros_sim_generate_messages_cpp
	ascend_msgs_generate_messages_cpp)

add_executable(fuser src/fuser.cpp)
target_link_libraries(fuser libAI ${catkin_LIBRARIES})
add_dependencies(fuser planning_ros_sim_generate_messages_cpp
	ascend_msgs_generate_messages_cpp)

# Test
catkin_add_gtest(Plank src/test/plank_test.cpp)
catkin_add_gtest(World src/test/world_test.cpp)
catkin_add_gtest(Drone src/test/drone_test.cpp)
catkin_add_gtest(Robot src/test/robot_test.cpp)
catkin_add_gtest(AI src/test/ai_test.cpp)
catkin_add_gtest(Fuser src/test/fuser_test.cpp)

target_link_libraries(Plank libAI ${catkin_LIBRARIES})
target_link_libraries(Drone libAI ${catkin_LIBRARIES})
target_link_libraries(Robot libAI ${catkin_LIBRARIES})
target_link_libraries(AI libAI ${catkin_LIBRARIES})
target_link_libraries(Fuser libAI ${catkin_LIBRARIES})

# %EndTag(FULLTEXT)%
