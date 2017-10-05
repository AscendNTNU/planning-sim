#!/bin/bash

cd catkin_ws
catkin_make

source devel/setup.bash
roslaunch planning-sim run_nodes.launch
