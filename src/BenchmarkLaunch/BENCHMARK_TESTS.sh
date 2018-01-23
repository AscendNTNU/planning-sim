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
if [ $? -eq 0 ]; then
    echo COMPILED SUCCESS
    xterm -title 'App1' -hold -e 'roscore' &
	xterm -title 'App2' -hold -e 'rosrun planning_ros_sim benchmark_perception_control' &
	xterm -title 'App3' -hold -e 'rosrun planning_ros_sim planning'
else
    echo COMPILED ERROR
    $SHELL
fi