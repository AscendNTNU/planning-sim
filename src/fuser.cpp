#include "fuser.h"

std::array<Robot, 10> robots_in_memory;
std::vector<std::vector<Robot>> observed_robots;
std::array<Robot, 4> obstacle_robots_in_memory;
std::vector<std::vector<Robot>> observed_obstacle_robots;

World world = World(0);

ros::Time start_time(0.0);
float elapsed_time = 0.0; // This is set by a callback if we are using ai-sim

float TIMEOUT_OBSERVATION = 0.01;

point_t drone_position = point_zero;

void initializeRobotsInMemory(){
    for(int i=0;i<10;i++){
        float t = 3.14*2.0 * i / 10.0;
        point_t point;
        point.x = 10.0 + cosf(t);
        point.y = 10.0 + sinf(t);
        float orientation = t;
        robots_in_memory[i].update(i,point,orientation, 0, false);
    }
}

void groundRobotCallback(ascend_msgs::DetectedRobotsGlobalPositions::ConstPtr msg){
    std::vector<Robot> robots_seen_in_one_message;
    std::vector<Robot> obstacle_robots_seen_in_one_message;
    for(int i = 0; i < (int)msg->count; i++) {
        Robot robot;

        point_t position;
        position.x = msg->global_robot_position[i].x;
        position.y = msg->global_robot_position[i].y;

        float q = msg->direction[i];
        float time = msg->header.stamp.sec-start_time.sec;
        bool visible = true;

        robot.update(i, position, q , time, visible);
        if(msg->robot_color[i]!=3){
            robots_seen_in_one_message.push_back(robot);
        }else{
            obstacle_robots_seen_in_one_message.push_back(robot);
        }        
    }
    observed_robots.push_back(robots_seen_in_one_message);
    observed_obstacle_robots.push_back(obstacle_robots_seen_in_one_message);
}

void startTimeCallback(std_msgs::Time::ConstPtr msg){
    start_time = msg->data;
}

void dronePositionCallback(geometry_msgs::PoseStamped::ConstPtr msg){
    drone_position.x = msg->pose.position.x;
    drone_position.y = msg->pose.position.y;
    drone_position.z = msg->pose.position.z;
}

void aiSimCallback(ascend_msgs::AIWorldObservation::ConstPtr obs){

    elapsed_time = obs->elapsed_time;

    drone_position.x = obs->drone_position.x;
    drone_position.y = obs->drone_position.y;
    drone_position.z = obs->drone_position.z;

    int i = -1;
    std::vector<Robot> robots_seen_in_one_message;
    for(auto it = obs->ground_robots.begin(); it != obs->ground_robots.end(); it++, i++) {
        if(it->visible){
            Robot robot;

            point_t position;
            position.x = it->x;
            position.y = it->y;
            float q = it->theta;
            float time = elapsed_time;
            bool visible = true;

            robot.update(i, position, q , time, visible);
            robots_seen_in_one_message.push_back(robot);
        }
    }
    observed_robots.push_back(robots_seen_in_one_message);

    i = -1;
    std::vector<Robot> obstacle_robots_seen_in_one_message;
    for(auto it = obs->obstacle_robots.begin(); it != obs->obstacle_robots.end(); it++, i++){
        if(it->visible){

            Robot robot;
            point_t position;
            position.x = it->x;
            position.y = it->y;
            float q = it->theta;
            float time = elapsed_time;
            bool visible = true;

            robot.update(i, position, q , time, visible);
            obstacle_robots_seen_in_one_message.push_back(robot);
        }
    }
    observed_obstacle_robots.push_back(robots_seen_in_one_message);
}

void updateRobots(std::vector<Robot> robots_in_single_message, float current_time){
    std::set<int> used_indices;

    for(auto it = robots_in_single_message.begin(); it != robots_in_single_message.end(); it++){

        Robot new_robot_observation = *it;
        int nearest_robot_index = nearestNeighbor(new_robot_observation, used_indices);

        if(nearest_robot_index >= 0){
            robots_in_memory[nearest_robot_index].update(new_robot_observation);
            used_indices.insert(nearest_robot_index);
        }
    }

    for(auto it = used_indices.begin(); it != used_indices.end(); it++){
        robots_in_memory[*it].kalmanStepNoObservation(current_time);
    }
}

float calcCurrentTime(float seconds){
    if(elapsed_time == 0.0){
        return seconds-start_time.sec; 
    }
    return elapsed_time;
}

int main(int argc, char **argv){

    int counter = 0;
    // Initialize ros-messages
    ros::init(argc, argv, "fuser");

    initializeRobotsInMemory();

    ros::NodeHandle node;
    // geometry_msgs::Pose2D drone_msg;
    std_msgs::Float32 time_msg;

    ros::Subscriber tracker_sub = node.subscribe("globalGroundRobotPosition", 100, groundRobotCallback);
    ros::Subscriber start_time_sub = node.subscribe("/time_chatter/start_time", 1, startTimeCallback);
    ros::Subscriber drone_sub = node.subscribe("/mavros/local_position/pose", 1, dronePositionCallback);
    ros::Subscriber sim_sub = node.subscribe("/ai/sim", 1, aiSimCallback);

    ros::Publisher observation_pub = node.advertise<ascend_msgs::AIWorldObservation>("AIWorldObservation", 1);

    ros::Rate rate(20);

    while (ros::ok()) {
        ros::spinOnce();

        if(elapsed_time == 0.0 && start_time.sec == 0.0){
            continue;
        }
        
        ascend_msgs::AIWorldObservation observation;

        float current_time = calcCurrentTime(ros::Time::now().sec);
        observation.elapsed_time = current_time;

        for(auto it = observed_robots.begin(); it != observed_robots.end(); it++){
            updateRobots(*it, current_time);
        }
        for(auto it = observed_obstacle_robots.begin(); it != observed_obstacle_robots.end(); it++){
            updateRobots(*it, current_time);
        }


        for(int i=0; i<robots_in_memory.size(); i++){
            ascend_msgs::GRState robot;

            Robot robot_at_current_time = robots_in_memory[i].getRobotPositionAtTime(current_time);
            
            robot.x = robots_in_memory[i].x_hat_k.at<double>(1,1);
            robot.y = robots_in_memory[i].x_hat_k.at<double>(2,1);
            robot.theta = robots_in_memory[i].x_hat_k.at<double>(3,1);

            if(observation.elapsed_time - robots_in_memory[i].getTimeLastSeen() > TIMEOUT_OBSERVATION){
                robots_in_memory[i].setVisible(false);
            }

            robot.visible = robots_in_memory[i].getVisible();
            observation.ground_robots[i] = robot;
        }

        for(int i=0; i<obstacle_robots_in_memory.size(); i++){
            ascend_msgs::GRState robot;

            Robot robot_at_current_time = obstacle_robots_in_memory[i].getRobotPositionAtTime(current_time);
            
            robot.x = obstacle_robots_in_memory[i].x_hat_k.at<double>(1,1);
            robot.y = obstacle_robots_in_memory[i].x_hat_k.at<double>(2,1);
            robot.theta = obstacle_robots_in_memory[i].x_hat_k.at<double>(3,1);

            if(observation.elapsed_time - obstacle_robots_in_memory[i].getTimeLastSeen() > TIMEOUT_OBSERVATION){
                obstacle_robots_in_memory[i].setVisible(false);
            }

            robot.visible = obstacle_robots_in_memory[i].getVisible();
            observation.obstacle_robots[i] = robot;
        }

        geometry_msgs::Point32 drone;
        drone.x = drone_position.x;
        drone.y = drone_position.y;
        drone.z = drone_position.z;
        observation.drone_position = drone;
        observation.header.seq = 1;
        observation_pub.publish(observation);
        rate.sleep();
    }

    return 0;

}
