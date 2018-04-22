#!/bin/bash
echo "Run PLanning"
echo "If planning-ros-sim not found, close and open terminal and run again"

ROS_HOST=http://localhost
ROS_PORT=$((11311 + (RANDOM % 1000) + 1))
export ROS_MASTER_URI=$ROS_HOST:$ROS_PORT

if [ -z "$IS_DOCKER" ]; then
	killall -9 roscore
	killall -9 rosmaster
	killall sim
	killall sim_no_gui
fi

WORKDIR=$(pwd)
cd ../../../..
if [ -z "$IS_DOCKER" ]; then
	xterm -title 'build' -e 'catkin_make'
else
	WORKDIR=/catkin_ws/src/planning-sim/src/BenchmarkLaunch
	cd /catkin_ws
	catkin_make
	source devel/setup.bash
fi
cd $WORKDIR
cd ..

#g++ ai-sim/sim_no_gui.cpp -o sim_no_gui -lGL `sdl2-config --cflags --libs`

defaultNumRunsPerSeed=6
defaultNumSeeds=100

if [ "$1" == "multiple" ]; then
	MULTIPLE=1
	numRunsPerSeed="${3:-$defaultNumRunsPerSeed}"
	numSeeds="${2:-$defaultNumSeeds}"
else
	numRunsPerSeed="${2:-$defaultNumRunsPerSeed}"
	numSeeds="${1:-$defaultNumSeeds}"
fi

if [ -n "$RUNS_PER_SEED" ]; then
	numRunsPerSeed=$RUNS_PER_SEED
fi

if [ -n "$SEEDS" ]; then
	numSeeds=$SEEDS
fi

totalRobotsOut=0
numRuns=$(($numRunsPerSeed*$numSeeds))
numRun=0
wins=0
winsVisual=""
echo "Num Runs: $numRuns"

if [ -z "$IS_DOCKER" ]; then
	xterm -title 'roscore' -e 'roscore' &
else
	roscore -p $ROS_PORT &
fi

function updateProgress () {
	if [ -n "$IS_DOCKER" ]; then
		numRuns="$1"
		numRun="${2:-0}"
		winsVisual="$3"
		totalRobotsOut="$4"

		if [ $numRuns != $numRun ]; then
			left=$(printf "%0.s." $(seq 1 $(awk "BEGIN{print $numRuns-$numRun}")))
		else
			left=""
		fi
		percent=$(awk "BEGIN{print $numRun/$numRuns*100}")
		average=$(awk 'BEGIN{printf "%.2f", '$totalRobotsOut'/'$numRun'}')

		if [ -n "$MULTIPLE" ]; then
			echo "[$winsVisual$left] ($percent%)  Avg=$average"
		else
			echo -ne '['$winsVisual$left'] ('$percent'%)  Avg='$average'     \r'
		fi
	fi
}

startTime=`date +%s`
if [ $? -eq 0 ]; then
	echo COMPILED SUCCESS
	(sleep 2 && updateProgress $numRuns) &

	for j in $(seq 1 $numSeeds)
	do
		[ -z $IS_DOCKER ] && echo "Seed: $j"
		for i in $(seq 1 $numRunsPerSeed);
		do
			statusfile1=$(mktemp)
			cmd="rosrun planning_ros_sim benchmark_perception_control $j"

			if [ -z "$IS_DOCKER" ]; then
				xterm -title 'Perception_Control' -e $cmd &
				# xterm -title 'Perception_Control' -e sh -c 'rosrun planning_ros_sim benchmark_perception_control 744; echo $? > '$statusfile2 &
				# xterm -title 'Planning' -hold -e 'rosrun planning_ros_sim planning'
				xterm -title 'Planning' -e sh -c 'rosrun planning_ros_sim planning; echo $? > '$statusfile1
			else
				$cmd > /dev/null &
				rosrun planning_ros_sim planning > /dev/null; echo $? > $statusfile1
			fi

			robotsOut=$(cat $statusfile1)
			totalRobotsOut=$((totalRobotsOut+robotsOut))
			rm $statusfile1
			[ -z $IS_DOCKER ] && echo "Robots Out: $robotsOut"
			numRun=$((numRun+1))

			if [ $robotsOut -gt 3 ]; then
				wins=$((wins+1))
				winsVisual=$winsVisual"="
				[ -z $IS_DOCKER ] && echo "wins: $wins"
			else
				winsVisual=$winsVisual"X"
			fi

			updateProgress $numRuns $numRun $winsVisual $totalRobotsOut
		done
	done
else
	echo COMPILED ERROR
	$SHELL
fi

if [ -z "$IS_DOCKER" ]; then
	average=$(echo "scale=5;$totalRobotsOut/$numRuns" | bc)
	winRate=$(echo "scale=5;$wins/$numRuns" | bc)
else
	average=$(awk 'BEGIN{printf "%.2f", '$totalRobotsOut'/'$numRuns'}')
	winRate=$(awk 'BEGIN{printf "%.2f", '$wins'/'$numRuns'}')
	echo "[$winsVisual] (100%)  Avg=$average     "
fi

echo "Total Robots Out: $totalRobotsOut"
echo "Average Robots Out: $average"
echo "Win Rate: $winRate"

endTime=`date +%s`
deltaTime=$((endTime-startTime))
avgTime=$(awk 'BEGIN{printf "%.2f", '$deltaTime'/'$numRuns'}')

if [ -n "$IS_DOCKER" ]; then
	winRatePercent=$(awk "BEGIN{print $winRate*100}")
	printf "TotRobOut=%-5s AvgRobOut=%-6s Time=%-6s AvgTime=%-8s History: %s\n" $totalRobotsOut $average $deltaTime"s" $avgTime"s" "[$winsVisual] ($wins/$numRuns=$winRatePercent%)"
fi
