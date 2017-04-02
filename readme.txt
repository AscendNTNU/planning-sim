Add this in your catkin_ws/src and it's ready to run.
The nodes doesn't start until the gui is running. Start the gui from catkin_ws/src/planning_ros_sim/src/ai-sim/COMPILE_AND_RUN_SIM.sh
It's important that the gui is started from inside the planning_ros_sim folder.

TO DO:
- some of the files needed for gui (such as sim.h) are saved twice. Delete does who is excessive and make sure they're still included correct.
- Merge the algorithm with this ros code. The algorithm should be run from only the planning node.
- Use actionlib?
- Find all the bugs and solve them
