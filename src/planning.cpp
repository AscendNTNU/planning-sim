#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "planning_ros_sim/groundRobotList.h"
#include "planning_ros_sim/groundRobot.h"
#include "planning_ros_sim/droneCmd.h"
#include <actionlib/client/simple_action_client.h>
#include <stdio.h>
#include "AI/AI.h"
#include "AI/structs.h"
#include <assert.h>

const float SIMILARITY_THRESHOLD = 1;

planning_ros_sim::groundRobotList GroundRobots;
geometry_msgs::Pose2D Drone;

float elapsed_time = 0;

bool action_done = true;

AI ai = AI();
World world = World(0);
State state = State();

void time_chatterCallback(std_msgs::Float32 msg) {
    elapsed_time = (float)msg.data;
}

void groundRobot_chatterCallback(const planning_ros_sim::groundRobotList &msg) {
    observation_t robotObs = observation_Empty;
    for(int i = 0; i < 10; i++) {
          robotObs.robot_x[i] = msg.groundRobot[i].x;
          robotObs.robot_y[i] = msg.groundRobot[i].y;
          robotObs.robot_q[i] = msg.groundRobot[i].theta;
    }
    state.updateRobot(robotObs, elapsed_time);
}

void drone_chatterCallback(geometry_msgs::Pose2D msg) {
    observation_t droneObs = observation_Empty;
    droneObs.drone_x = msg.x;
    droneObs.drone_y = msg.y;
    state.updateDrone(droneObs, elapsed_time);
}

void command_done_chatterCallback(std_msgs::Bool msg) {
    action_done = (bool)msg.data;
}

planning_ros_sim::droneCmd to_ROS_Command(action_t action) {
    planning_ros_sim::droneCmd command;
    command.x = action.where_To_Act.x;
    command.y = action.where_To_Act.y;
    command.z = 0;
    command.cmd = (int)action.type;
    command.target_id = action.target;
    command.reward = action.reward;

    return command;
}

bool is_nearby(point_t current_Where_To_Act, point_t target) {
      double x1 = current_Where_To_Act.x;
      double y1 = current_Where_To_Act.y;
      double x2 = target.x;
      double y2 = target.y;
      
      double dist = pow(pow(x2-x1,2) + pow(y2-y1,2), .5);
      return dist < SIMILARITY_THRESHOLD;
}

float similarity(action_t action1 ,action_t action2) {
    if(is_nearby(action1.where_To_Act, action2.where_To_Act)) {
        return 1;
    }
    return 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "planning");
    ros::NodeHandle ground_robot_node;
    ros::NodeHandle drone_node;
    ros::NodeHandle command_node;
    ros::NodeHandle time_node;
    ros::NodeHandle command_done_node;

    ros::Subscriber time_sub = time_node.subscribe("time_chatter", 1000, time_chatterCallback);
    ros::Subscriber ground_robot_sub = ground_robot_node.subscribe("groundrobot_chatter", 1000, groundRobot_chatterCallback);
    ros::Subscriber drone_sub = drone_node.subscribe("drone_chatter", 1000, drone_chatterCallback);
    ros::Subscriber command_done_sub = command_done_node.subscribe("command_done_chatter", 100, command_done_chatterCallback);
    
    ros::Publisher command_pub = command_node.advertise<planning_ros_sim::droneCmd>("drone_cmd_chatter", 1000);

    planning_ros_sim::droneCmd drone_action;
    
    int target_id = 0;
    Robot target = state.getRobot(1);
    action_t current_action;
    action_t updated_action;
    std::stack<action_t> current_action_stack;

    world.startTimer();

    while (ros::ok() and state.getRobot(0).getPosition().x == 0) {
        ros::Duration(0.2).sleep();
        ros::spinOnce();
    }

    while (ros::ok()) {
        ros::Duration(0.4).sleep();
        ros::spinOnce();

        if(action_done && 2.5 < fmod(elapsed_time, 20) && fmod(elapsed_time, 20) < 17.5 ) {
            // target = state.getRobot(target_id);

            //If we've finished our stack get a new one.
            if(current_action_stack.empty()) {
                current_action_stack = ai.getBestGeneralActionStack(state);
            }

            current_action = current_action_stack.top();
            target = state.getRobot(current_action.target);

            if(!target.isMoving()){
                while (!current_action_stack.empty()) {
                    current_action_stack.pop();
                }
                continue;
            }
            // If we are waiting on the ground robot(ie the robot isn't
            // nearby our landing location) we might aswell update our
            // where_to_act on our current observations.
            if(current_action.type != search && !is_nearby(current_action.where_To_Act, target.getPosition())) {
                
                updated_action = ai.chooseAction(target, state.getDrone());
                updated_action.type = search;

                if(!similarity(updated_action, current_action)) {

                    while (!current_action_stack.empty()) {
                        current_action_stack.pop();
                    }
                    continue;
                }
                //update where to act and go to new search pos
                current_action = updated_action;
                current_action_stack.push(updated_action);

            }

            drone_action = to_ROS_Command(current_action);
            command_pub.publish(drone_action);
            current_action_stack.pop();
        }
    }

    return 0;
}


// while(ok) {
// if(no actions and not near turn time) { //Get land_on_top/land_in_front at point
//     get action stack
//     if target is not moving {
//         delete actions
//         go to beginning
//     }
// }
// else if(search done and target is not here) { //update that plank point based on robots movements
//     if target is moving
//         get new search action
//         if not similar
//             delete actions
//             go to beginning
//     else
//         go to beginning
// }
// execute action
// }
