# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(planning_ros_sim)

## Find catkin and any catkin packages + message_generation
find_package(
	catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs 
	geometry_msgs 
	message_generation 
	actionlib 
	actionlib_msgs 
	ascend_msgs
	visualization_msgs
)
find_package( OpenCV REQUIRED )
message( STATUS "OpenCV Include Dir: " ${OpenCV_INCLUDE_DIRS} )
## Declare ROS messages and services

#add_service_files(DIRECTORY srv FILES)

add_message_files(
	FILES 
	groundRobot.msg 
	groundRobotList.msg 
	droneCmd.msg
)

## Generate added messages and services
generate_messages(
	DEPENDENCIES 
	std_msgs 
	geometry_msgs
)

## Declare a catkin package
catkin_package(
	CATKIN_DEPENDS 
	message_runtime 
	roscpp 
	ascend_msgs
	LIBRARIES 
	libAI
	roscpp 
	ascend_msgs
	visualization_msgs
)

file(GLOB planning_tools_SRC "src/tools/*.cpp")
add_library(planning_tools_lib
	${planning_tools_SRC}
)

file(GLOB libAI_SRC "src/AI/*.cpp")
add_library(libAI
	${libAI_SRC}
)

target_link_libraries(libAI planning_tools_lib)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
include_directories(include ${catkin_INCLUDE_DIRS})

set(CMAKE_BUILD_TYPE Debug)

if(NOT ($ENV{ASCEND_ENV} MATCHES "^production$"))
	# ============= Perception / Control ===============
	add_executable(
		perception_control src/perception_control.cpp
	)
	target_link_libraries(
		perception_control planning_tools_lib ${catkin_LIBRARIES}
	)
	add_dependencies(
		perception_control planning_ros_sim_generate_messages_cpp
	)
	# ==================== Benchmark ===================
	add_executable(
		benchmark_perception_control src/benchmark_perception_control.cpp
	)
	target_link_libraries(
		benchmark_perception_control planning_tools_lib ${catkin_LIBRARIES}
	)
	add_dependencies(
		benchmark_perception_control planning_ros_sim_generate_messages_cpp
	)
endif()

# ================== Planning ====================
add_executable(
	planning src/planning.cpp
)
target_link_libraries(
	planning libAI ${catkin_LIBRARIES}
)
add_dependencies(
	planning planning_ros_sim_generate_messages_cpp
	ascend_msgs_generate_messages_cpp
)


#include <opencv2/opencv.hpp>
# ==================== Fuser =====================
add_executable(
	fuser src/fuser.cpp
)
target_link_libraries(
	fuser libAI ${catkin_LIBRARIES}
)
add_dependencies(
	fuser planning_ros_sim_generate_messages_cpp ascend_msgs_generate_messages_cpp
)

# ================= Visualizer ==================
add_executable(
	visualizer src/RViz_nodes/AIWorldObservationVisualization.cpp
)
add_dependencies(
	visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
	visualizer ${catkin_LIBRARIES}
)

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
target_link_libraries(libAI ${OpenCV_LIBRARIES})
target_link_libraries(Fuser libAI ${catkin_LIBRARIES})

# %EndTag(FULLTEXT)%
