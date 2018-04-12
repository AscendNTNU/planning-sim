#include "fuser.h"

const bool USE_FUSER = false;

std::vector<Robot> robots_in_memory (10);
std::vector<std::vector<Robot>> observed_robots;
std::vector<Robot> obstacle_robots_in_memory (4);
std::vector<std::vector<Robot>> observed_obstacle_robots;

World world = World(0);

ros::Time start_time(0.0);
float elapsed_time = 0.0; // This is set by a callback if we are using ai-sim

float TIMEOUT_OBSERVATION = 0.1;

point_t drone_position = point_zero;

// Callbacks
void groundRobotCallback(ascend_msgs::DetectedRobotsGlobalPositions::ConstPtr msg){
    std::vector<Robot> robots_seen_in_one_message;
    std::vector<Robot> obstacle_robots_seen_in_one_message;
    for(int i = 0; i < (int)msg->count; i++) {
        Robot robot;

        point_t position;
        position.x = msg->global_robot_position.at(i).x;
        position.y = msg->global_robot_position.at(i).y;

        float q = msg->direction.at(i);
        float time = msg->header.stamp.sec-start_time.sec;
        bool visible = true;

        robot.update(i, position, q , time, visible);
        if(msg->robot_color.at(i)!=3){
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

    int i = 0;
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

    i = 0;
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
    observed_obstacle_robots.push_back(obstacle_robots_seen_in_one_message);
}


//Helper functions
void initializeRobotsInMemory(){
    for(int i=0;i<robots_in_memory.size();i++){

        // The robots spawn in a circle,
        // but at an initial radius of 1 meters.
        float t = 3.14*2.0 * i / (float)robots_in_memory.size();
        point_t point;
        point.x = 10.0 + cosf(t);
        point.y = 10.0 + sinf(t);
        float orientation = t;
        robots_in_memory.at(i).update(i,point,orientation, 0, false);
    }

    for (unsigned int i = 0; i < obstacle_robots_in_memory.size(); i++){
        float t =3.14*2.0 * i / (float)obstacle_robots_in_memory.size();

        // The obstacles are also spawned in a circle,
        // but at an initial radius of 5 meters.
        point_t point;
        point.x = 10.0f + 5 * cosf(t);
        point.y = 10.0f + 5 * sinf(t);
        float orientation = t;
        obstacle_robots_in_memory.at(i).update(i, point, orientation, 0, false);
    }
}

void updateRobots(std::vector<Robot> robots_in_single_message, std::vector<Robot> &memory, float current_time){
    std::set<int> used_indices;
    for(auto it = robots_in_single_message.begin(); it != robots_in_single_message.end(); it++){
        Robot new_robot_observation = *it;
        int nearest_robot_index = nearestNeighbor(new_robot_observation, memory, used_indices);

        if(nearest_robot_index >= 0){
            memory.at(nearest_robot_index).update(new_robot_observation);
            used_indices.insert(nearest_robot_index);
        }
    }

    // for(auto it = used_indices.begin(); it != used_indices.end(); it++){
    //     memory.at(*it).kalmanStepNoObservation(current_time);
    // }
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

    ros::Subscriber tracker_sub = node.subscribe("globalGroundRobotPosition", 10, groundRobotCallback);
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


        if(USE_FUSER){
            for(auto it = observed_robots.begin(); it != observed_robots.end(); it++){
                updateRobots(*it, robots_in_memory, current_time);
            }
            for(auto it = observed_obstacle_robots.begin(); it != observed_obstacle_robots.end(); it++){
                updateRobots(*it, obstacle_robots_in_memory, current_time);
            }
        }

        else{
            int i = 0;
            std::vector<Robot> last_observation = *observed_robots.begin();
            for(auto it = last_observation.begin(); it != last_observation.end(); it++){
                robots_in_memory.at(i).update(*it);
                i++;
            }

            i = 0;
            std::vector<Robot> last_obstacle_observation = *observed_obstacle_robots.begin();
            for(auto it = last_obstacle_observation.begin(); it != last_obstacle_observation.end(); it++){
                obstacle_robots_in_memory.at(i).update(*it);
                i++;
            }
        }

        observed_robots.clear();
        observed_obstacle_robots.clear();

        for(int i=0; i<robots_in_memory.size(); i++){
            ascend_msgs::GRState robot;

            // robot.x = robots_in_memory.at(i).x_hat_k.at<double>(1,1);
            // robot.y = robots_in_memory.at(i).x_hat_k.at<double>(2,1);
            // robot.theta = robots_in_memory.at(i).x_hat_k.at<double>(3,1);
            robot.x = robots_in_memory.at(i).getPosition().x;
            robot.y = robots_in_memory.at(i).getPosition().y;
            robot.theta = robots_in_memory.at(i).getOrientation();

            if(observation.elapsed_time - robots_in_memory.at(i).getTimeLastSeen() > TIMEOUT_OBSERVATION){
                robots_in_memory.at(i).setVisible(false);
            }

            robot.visible = robots_in_memory.at(i).getVisible();
            observation.ground_robots.at(i) = robot;
        }

        for(int i=0; i<obstacle_robots_in_memory.size(); i++){
            ascend_msgs::GRState robot;

            robot.x = obstacle_robots_in_memory.at(i).getPosition().x;
            robot.y = obstacle_robots_in_memory.at(i).getPosition().y;
            robot.theta = obstacle_robots_in_memory.at(i).getOrientation();

            if(observation.elapsed_time - obstacle_robots_in_memory.at(i).getTimeLastSeen() > TIMEOUT_OBSERVATION){
                obstacle_robots_in_memory.at(i).setVisible(false);
            }

            robot.visible = obstacle_robots_in_memory.at(i).getVisible();
            observation.obstacle_robots.at(i) = robot;
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
