# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(planning_ros_sim)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
find_package(Eigen3 REQUIRED)
find_package( OpenCV REQUIRED )

#find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

## Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES Num.msg)
#add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

# %EndTag(FULLTEXT)%
