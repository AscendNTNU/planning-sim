Add this in your catkin_ws/src and it's ready to run.
The nodes doesn't start until the gui is running. Start the gui from catkin_ws/src/planning_ros_sim/src/ai-sim/COMPILE_AND_RUN_SIM.sh
It's important that the gui is started from inside the planning_ros_sim folder.


Linux users must run 'sudo apt-get install libsdl2-dev' at least once to get the SDL2 libraries.

Be sure to run 'source devel/setup.bash' in every terminal, or alternatively add 
source [path to catkin_ws]/catkin_ws/devel/setup.bash
to your ~/.bashrc file (hidden file in the home directory, can be opened from terminal with gedit/nano/etc, for example 'gedit .bashrc' when in home or ~ directory)

Terminal window 1:
Navigate to the folder where COMPILE_AND_RUN_SIM.sh is stored (as of this writing planning-sim/src/ai-sim) and type into the terminal: 
bash COMPILE_AND_RUN_SIN.sh

Terminal window 2:
Navigate to the catkin_ws directory, and run:
catkin_make

Fix any build errors before continuing.

Terminal window 3:
Run: 
rosrun planning_ros_sim perception_control

Terminal window 4:
Run:
rosrun planning_ros_sim planning

