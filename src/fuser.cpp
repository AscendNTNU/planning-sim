#include "fuser.h"

const bool USE_FUSER = true;
const int NUMBER_OF_ROBOTS = 10;
const double SAFE_VISIBILITY_RADIUS = 3;
const double TIMEOUT_ROBOT_NOT_VISIBLE = 40;
const double TIMEOUT_ROBOT_SHOULD_BE_VISIBLE = 5;

std::set<int> set_of_indices;

std::vector<Robot> robots_in_memory;
std::vector<std::vector<Robot>> observed_robots;
std::vector<Robot> obstacle_robots_in_memory (4);
std::vector<std::vector<Robot>> observed_obstacle_robots;

World world = World(0);

ros::Time start_time(0.0);
double simulation_time = 0.0; // This is set by a callback if we are using ai-sim
point_t drone_position = point_zero;

// Callbacks
void aiSimCallback(ascend_msgs::AIWorldObservation::ConstPtr obs){ 
   
    simulation_time = obs->elapsed_time;  
   
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
            float time = simulation_time; 
            bool visible = true;   
            robot.setSideCamera(false);
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
            float time = simulation_time; 
            bool visible = true;   
   
            robot.update(i, position, q , time, visible);  
            obstacle_robots_seen_in_one_message.push_back(robot);  
        }  
    }  
    observed_obstacle_robots.push_back(obstacle_robots_seen_in_one_message);   
}

void groundRobotCallback(ascend_msgs::DetectedRobotsGlobalPositions::ConstPtr msg){
    
    if(drone_position.z < 0.4){
       return; 
    }

    std::vector<Robot> robots_seen_in_one_message;
    std::vector<Robot> obstacle_robots_seen_in_one_message;
    for(int i = 0; i < (int)msg->count; i++) {


        // header.stamp.sec == 0 if not set
        // If header.stamp is not set, the filter will fail
        if(msg->header.stamp.sec < 1){
            continue;
        }

        Robot robot;

        point_t position;
        position.x = msg->global_robot_position.at(i).x;
        position.y = msg->global_robot_position.at(i).y;
        double q = msg->direction.at(i);
        double time = ros::Time::now().toSec() - start_time.toSec();

        bool visible = true;

        robot.update(i, position, q , time, visible);
        robots_seen_in_one_message.push_back(robot);
        
        if(msg->robot_color.at(i)!=3){
                robots_seen_in_one_message.push_back(robot);
        }else{
                obstacle_robots_seen_in_one_message.push_back(robot);
        }

        if(msg->camera_type.at(i) == 0){
            robot.setSideCamera(false);
        }
        else{
            robot.setSideCamera(true);
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

//Helper functions
void initializeFuser(){
    for(int i=0; i<NUMBER_OF_ROBOTS; i++){
        set_of_indices.emplace(i);
    }

    for(int i=0;i<NUMBER_OF_ROBOTS;i++){
        // The robots spawn in a circle,
        // but at an initial radius of 1 meters.
        robots_in_memory.push_back(Robot(i));

        double t = 3.14*2.0 * i / (double)robots_in_memory.size();
        point_t point;
        point.x = 10.0 + cosf(t);
        point.y = 10.0 + sinf(t);
        double orientation = t;
        robots_in_memory.at(i).update(i,point,orientation, 0, false);
    }

    for (unsigned int i = 0; i < obstacle_robots_in_memory.size(); i++){
        double t =3.14*2.0 * i / (double)obstacle_robots_in_memory.size();

        // The obstacles are also spawned in a circle,
        // but at an initial radius of 5 meters.
        point_t point;
        point.x = 10.0f + 5 * cosf(t);
        point.y = 10.0f + 5 * sinf(t);
        double orientation = t;
        obstacle_robots_in_memory.at(i).update(i, point, orientation, 0, false);
    }
}

std::set<int> updateRobots(std::vector<Robot> robots_in_single_message, std::vector<Robot> &memory, double current_time){
    std::set<int> not_updated_indices = set_of_indices;
    for(auto it = robots_in_single_message.begin(); it != robots_in_single_message.end(); it++){
        Robot new_robot_observation = *it;

        bool robot_in_safe_vis_radius = (getDistanceBetweenPoints(new_robot_observation.getPosition(), drone_position) < SAFE_VISIBILITY_RADIUS);

        int nearest_robot_index = nearestNeighbor(new_robot_observation, memory, not_updated_indices, robot_in_safe_vis_radius);
        if(nearest_robot_index >= 0){
            memory.at(nearest_robot_index).update(new_robot_observation);
            not_updated_indices.erase(nearest_robot_index);
        }
    }
    return not_updated_indices;
}

void fuser_tick(std::vector<Robot>& memory, double current_time){
    std::set<int> not_observed_indices = set_of_indices;

    for(auto it = observed_robots.begin(); it != observed_robots.end(); it++){

            std::set<int> updated_indices;
            std::set<int> not_updated_indices = updateRobots(*it, robots_in_memory, current_time);

            std:set_intersection(not_observed_indices.begin(), not_observed_indices.end(),
                                 not_updated_indices.begin(), not_updated_indices.end(),
                                 std::inserter(updated_indices, updated_indices.begin()));

            not_observed_indices = updated_indices;
    }

    if(not_observed_indices.size() > 0){

        for(auto it = not_observed_indices.begin(); it != not_observed_indices.end(); it++){
            memory.at(*it).kalmanStepNoObservation(current_time);
        }
    }
}

bool isModelStillReliable(Robot robot, point_t drone_position, double current_time){

    //Different timeouts depending on if our model says the robot should be in sight or not

    double timeout = TIMEOUT_ROBOT_NOT_VISIBLE;

    if(getDistanceBetweenPoints(robot.getPosition(), drone_position) < SAFE_VISIBILITY_RADIUS){
        timeout = TIMEOUT_ROBOT_SHOULD_BE_VISIBLE;
    }

    if(current_time - robot.getTimeLastSeen() > timeout){
        return false;
    }

    // If a robot is out of the  arena, it should be removed
    
    if(!robot.isInArena()){
        return false;
    }

    return true;
}

ascend_msgs::AIWorldObservation createObservation(double current_time){
    ascend_msgs::AIWorldObservation observation;
    // Ground robots
    for(int i=0; i<robots_in_memory.size(); i++){
        ascend_msgs::GRState robot;
        robots_in_memory.at(i).setPositionToKalmanPosition();
        robot.x = robots_in_memory.at(i).getPosition().x;
        robot.y = robots_in_memory.at(i).getPosition().y;
        robot.theta = robots_in_memory.at(i).getOrientation();

        

        if(robots_in_memory.at(i).getVisible()){
            bool is_reliable = isModelStillReliable(robots_in_memory.at(i), drone_position, current_time);

            robots_in_memory.at(i).setVisible(is_reliable);
        }
        
        robot.visible = robots_in_memory.at(i).getVisible();
        observation.ground_robots.at(i) = robot;
    }

    geometry_msgs::Point32 drone;
    drone.x = drone_position.x;
    drone.y = drone_position.y;
    drone.z = drone_position.z;
    observation.drone_position = drone;
    observation.header.seq = 1;

    observation.elapsed_time = current_time;

    return observation;
}

double calcCurrentTime(ros::Time seconds){
    if(simulation_time == 0.0){
        return (seconds-start_time).toSec(); 
    }
    return simulation_time;
}

int main(int argc, char **argv){

    int counter = 0;
    // Initialize ros-messages
    ros::init(argc, argv, "fuser");

    initializeFuser();

    ros::NodeHandle node;
    std_msgs::Float32 time_msg;

    ros::Subscriber sim_sub = node.subscribe("/ai/sim", 1, aiSimCallback);

    ros::Subscriber tracker_sub = node.subscribe("globalGroundRobotPosition", 10, groundRobotCallback);
    ros::Subscriber start_time_sub = node.subscribe("/time_chatter/start_time", 1, startTimeCallback);
    ros::Subscriber drone_sub = node.subscribe("/mavros/local_position/pose", 1, dronePositionCallback);

    ros::Publisher observation_pub = node.advertise<ascend_msgs::AIWorldObservation>("AIWorldObservation", 1);

    ros::Rate rate(20);
    while (ros::ok()) {
        ros::spinOnce();

        if(simulation_time == 0.0 && start_time.toSec() == 0.0){
            continue;
        }
        
        double current_time = calcCurrentTime(ros::Time::now());

        if(USE_FUSER){
            fuser_tick(robots_in_memory, current_time);

        }
        else{

            int i = 0;
            if(observed_robots.size() != 0){
                std::vector<Robot> last_observation = *observed_robots.begin();
                
                for(auto it = last_observation.begin(); it != last_observation.end(); it++, i++){
                    
                    if(i >= NUMBER_OF_ROBOTS){
                        break;
                    }

                    robots_in_memory.at(i).update(*it);
                }
            }

            // i = 0;
            // if(observed_obstacle_robots.size() != 0){
            //     std::vector<Robot> last_obstacle_observation = *observed_obstacle_robots.begin();
            //     for(auto it = last_obstacle_observation.begin(); it != last_obstacle_observation.end(); it++){
            //         obstacle_robots_in_memory.at(i).update(*it);
            //         i++;
            //     }
            // }
        }

        observed_robots.clear();
        observed_obstacle_robots.clear();
        ascend_msgs::AIWorldObservation observation = createObservation(current_time);
        observation_pub.publish(observation);

        rate.sleep();
    }

    return 0;

}
