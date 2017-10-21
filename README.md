# Planning sim

Jenkins: [![Build Status](http://build.ascendntnu.no/buildStatus/icon?job=planning-sim)](http://build.ascendntnu.no/job/planning-sim)

Drone CI: 
[![Build Status](https://drone.ascendntnu.no/api/badges/AscendNTNU/planning-sim/status.svg)](https://drone.ascendntnu.no/AscendNTNU/planning-sim)

## Overview

This repo is an AI solution for mission 7a at IARC. The core concept is to give robots values based upon their position, orientation and the time until they turn. Once these values have been decided a multitude of classic AI algorithms can be run inorder to find the best action at a given time.  

Currently robot value is calculated by integrating the "plank" path the robot follows over a 2D reward space spanning an X and Y coordinate. The reward space has been calculated through value iteration.

The current algorithm is a greedy algorithm that finds the best placed robot based on the value of its plank. It then checks through all the possible actions and returns the best action based upon which action leads to the highest value plank.


## Setup

Add this repo in your `catkin_ws/src` and it's ready to run.

```bash
# Clone repo while adding all submodules recursivly
$ git clone --recursive git@github.com:ascendntnu/planning-sim.git
```

## Run GUI

The nodes doesn't start until the GUI is running. Start the GUI from `catkin_ws/src/planning-sim/src/ai-sim/COMPILE_AND_RUN_SIM.sh`.

```bash
# To update GUI submodule
$ git submodule update --init # If it is the first time and you did not clone with the --recursive flag
$ git submodule update # Every other time

# Go to directory
$ cd catkin_ws/src/planning-sim/src/ai-sim

# Run GUI
$ ./COMPILE_AND_RUN_SIM.sh # On mac: ./COMPILE_FOR_MAC_AND_RUN_SIM.sh
```

It's important that the GUI is started from inside the `planning-sim` folder.

Linux users must run `$ sudo apt-get install libsdl2-dev` at least once to get the SDL2 libraries.

## Run the AI (without Docker)

Be sure to run `$ source devel/setup.bash` in every terminal, or alternatively append 
`source {path/to/catkin_ws}/catkin_ws/devel/setup.bash`
to your `~/.bashrc` file using `$ echo "source {/some/absolute/path}/catkin_ws/devel/setup.bash" >> ~/.bashrc`.

Navigate to the `catkin_ws` directory, and run:

### Terminal window 1:

```bash
$ catkin_make

# Fix any build errors before continuing ...

$ run roscore
```

### Terminal window 2:

```bash
$ rosrun planning_ros_sim perception_control
```

### Terminal window 3:

```bash
$ rosrun planning_ros_sim planning
```

## Run the AI (with Docker)

Be sure to have `docker` and `docker-compose` (which should come with the docker installation).

```bash
$ docker-compose up nodes

# Or with GUI

$ xhost +local:root
$ docker-compose up nodes gui
```

## Add and run tests

Make tests in the `planning-sim/src/test` directory.

Add the test to the bottom of the `CMakeLists.txt` in the `planning-sim` directory (Same directory as this README.md).

```bash
# From catkin_ws run:
$ catkin_make run_tests

# Or, if you have Docker, run this from the repo:
$ docker-compose run tests
```
