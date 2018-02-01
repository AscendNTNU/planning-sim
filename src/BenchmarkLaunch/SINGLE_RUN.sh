#!/bin/bash
echo "Run PLanning"
echo "If planning-ros-sim not found, close and open terminal and run again"
killall -9 roscore
killall -9 rosmaster
killall sim
killall sim_no_gui
WORKDIR=$(pwd)
cd ../../../..
xterm -title 'build' -e 'catkin_make'
cd $WORKDIR
cd ..
#g++ ai-sim/sim_no_gui.cpp -o sim_no_gui -lGL `sdl2-config --cflags --libs`
numRuns=5
totalRobotsOut=0
xterm -title 'roscore' -e 'roscore' &
if [ $? -eq 0 ]; then
    echo COMPILED SUCCESS
	statusfile1=$(mktemp)
	statusfile2=$(mktemp)
	xterm -title 'Perception_Control' -hold -e sh -c 'rosrun planning_ros_sim benchmark_perception_control; echo $? > '$statusfile2 &
#	xterm -title 'Planning' -hold -e 'rosrun planning_ros_sim planning'
	xterm -title 'Planning' -hold -e sh -c 'rosrun planning_ros_sim planning; echo $? > '$statusfile1
	robotsOut=$(cat $statusfile1)
	status=$(cat $statusfile2)
	totalRobotsOut=$((totalRobotsOut+robotsOut))
	rm $statusfile1
	rm $statusfile2
	echo "Robots Out: $robotsOut"
	echo "Test: $status" 
else
    echo COMPILED ERROR
    $SHELL
fi
