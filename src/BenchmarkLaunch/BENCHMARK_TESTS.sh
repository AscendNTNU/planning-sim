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
numRunsPerSeed=5
numSeeds=100
totalRobotsOut=0
numRuns=$(($numRunsPerSeed*$numSeeds))
echo "Num Runs: $numRuns"
xterm -title 'roscore' -e 'roscore' &

if [ $? -eq 0 ]; then
	echo COMPILED SUCCESS
	for j in $(seq 1 $numSeeds)
	do
		echo "Seed: $j"
		for i in $(seq 1 $numRunsPerSeed);
		do
			statusfile1=$(mktemp)
			cmd="rosrun planning_ros_sim benchmark_perception_control $j"
			xterm -title 'Perception_Control' -e $cmd &
			# xterm -title 'Perception_Control' -e sh -c 'rosrun planning_ros_sim benchmark_perception_control 744; echo $? > '$statusfile2 &
		#	xterm -title 'Planning' -hold -e 'rosrun planning_ros_sim planning'
			xterm -title 'Planning' -e sh -c 'rosrun planning_ros_sim planning; echo $? > '$statusfile1
			robotsOut=$(cat $statusfile1)
			totalRobotsOut=$((totalRobotsOut+robotsOut))
			rm $statusfile1
			echo "Robots Out: $robotsOut"
		done
	done
else
    echo COMPILED ERROR
    $SHELL
fi
echo "Total Robots Out: $totalRobotsOut"
#AverageRobotsOut = $((totalRobotsOut/numRuns))
average=$(echo "scale=5;$totalRobotsOut/$numRuns" | bc)
echo "Average Robots Out: $average"


