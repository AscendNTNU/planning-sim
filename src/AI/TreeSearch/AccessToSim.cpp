#include "AccessToSim.h"
#include <array>

#define SIM_IMPLEMENTATION
#include "sim.h"

AccessToSim::AccessToSim(Observation observation) {
    std::array<sim_Position, Num_Targets> robots;
    std::array<sim_Position, Num_Obstacles> obstacles;
    sim_Position drone;

    for (int i = 0; i < Num_Robots; i++) {
    
        Robot robot = observation.getRobot(i);
    
        point_t pos = robot.getPosition();
        robots[i].x = pos.x;
        robots[i].y = pos.y;
        robots[i].q = robot.getOrientation();
    }

    for (int i = 0; i < Num_Obstacles; i++) {
        Robot obstacle = observation.getObstacle(i);
        point_t pos = obstacle.getPosition();

        obstacles[i].x = pos.x;
        obstacles[i].y = pos.y;
        obstacles[i].q = obstacle.getOrientation();
    }

    Drone drone_object = observation.getDrone();
    point_t pos = drone_object.getPosition();
    drone.x = pos.x;
    drone.y = pos.y;
    drone.q = pos.z;


    // std::cout << "Position of robot 0, from AccessToSim constructor" << std::endl;
    // std::cout << "x: " << robots[0].x << std::endl;
    // std::cout << "y: " << robots[0].y << std::endl;

    sim_State state = sim_init_state(observation.getTimeStamp(), drone, robots, obstacles);

    this->state = state;
}

Observation AccessToSim::simulateAction(action_t action){

    sim_Command cmd = convertToSimAction(action);
    this->state = sim_tick(this->state, cmd);
    cmd.type = sim_CommandType_NoCommand;
    while(!this->state.drone.cmd_done){
        for(int i = 0; i < 10; i++){
            this->state = sim_tick(this->state, cmd);
        }
    }
    Observation obs = getObservation();
    std::cout << "----Action is done simulating----" << std::endl;
    std::cout << "Elapsed time: " << obs.getTimeStamp() << std::endl;
    std::cout << "Position of robot 0, from simulateAction in AccessToSim" << std::endl;
    std::cout << "x: " << obs.getRobot(0).getPosition().x << std::endl;
    std::cout << "y: " << obs.getRobot(0).getPosition().y << std::endl;
    std::cout << "Position of drone, from simulateAction in AccessToSim" << std::endl;
    std::cout << "x: " << obs.getDrone().getPosition().x << std::endl;
    std::cout << "y: " << obs.getDrone().getPosition().y << std::endl;   

    return getObservation();
}

Observation AccessToSim::stepNoCommand() {
    sim_Command cmd;
    cmd.type = sim_CommandType_NoCommand;
    for(int i = 0; i < 10; i++){
        this->state = sim_tick(this->state, cmd);
    }
    return getObservation();
}

Observation AccessToSim::getObservation() {
    sim_Observed_State state = sim_observe_everything(this->state);

    observation_t new_observation;
    new_observation.elapsed_time = state.elapsed_time;
    new_observation.num_Targets = Num_Targets;
    new_observation.drone_x = state.drone_x;
    new_observation.drone_y = state.drone_y;
    new_observation.drone_cmd_done = state.drone_cmd_done;

    for (int i = 0; i < Num_Targets; i++) {
        new_observation.robot_x[i] = state.target_x[i];
        new_observation.robot_y[i] = state.target_y[i];
        new_observation.robot_q[i] = state.target_q[i];
    }

    for (int i = 0; i < Num_Obstacles; i++) {
        new_observation.obstacle_x[i] = state.obstacle_x[i];
        new_observation.obstacle_y[i] = state.obstacle_y[i];
        new_observation.obstacle_q[i] = state.obstacle_q[i];
    }

    Observation observation = Observation();
    observation.update(new_observation, state.elapsed_time);

    return observation;
}

observation_t AccessToSim::getObservationStruct() {
    sim_Observed_State state = sim_observe_everything(this->state);

    observation_t new_observation;
    new_observation.elapsed_time = state.elapsed_time;
    new_observation.num_Targets = Num_Targets;
    new_observation.drone_x = state.drone_x;
    new_observation.drone_y = state.drone_y;
    new_observation.drone_cmd_done = state.drone_cmd_done;

    for (int i = 0; i < Num_Targets; i++) {
        new_observation.robot_x[i] = state.target_x[i];
        new_observation.robot_y[i] = state.target_y[i];
        new_observation.robot_q[i] = state.target_q[i];
    }

    for (int i = 0; i < Num_Obstacles; i++) {
        new_observation.obstacle_x[i] = state.obstacle_x[i];
        new_observation.obstacle_y[i] = state.obstacle_y[i];
        new_observation.obstacle_q[i] = state.obstacle_q[i];
    }

    return new_observation;
}

Drone AccessToSim::getDrone() {
    observation_t observation = this->getObservationStruct();

    Drone drone = Drone();
    drone.update(observation);

    return drone;
}

std::array<Robot, 10> AccessToSim::getRobots() {
    observation_t observation = this->getObservationStruct();

    std::array<Robot, 10> robots;
    float elapsed_time = observation.elapsed_time;

    for (int i = 0; i < Num_Targets; i++) {
        Robot robot = Robot();
        point_t position;
        position.x = observation.robot_x[i];
        position.y = observation.robot_y[i];
        float orientation = observation.robot_q[i];
        robot.update(i, position, orientation, elapsed_time, true);

        robots[i] = robot;
    }

    return robots;
}

std::array<Robot, 4> AccessToSim::getObstacles() {
    observation_t observation = this->getObservationStruct();

    std::array<Robot, 4> obstacles;
    float elapsed_time = observation.elapsed_time;

    for (int i = 0; i < Num_Obstacles; i++) {
        Robot obstacle = Robot();
        point_t position;
        position.x = observation.robot_x[i];
        position.y = observation.robot_y[i];
        float orientation = observation.robot_q[i];
        obstacle.update(i, position, orientation, elapsed_time, true);

        obstacles[i] = obstacle;
    }

    return obstacles;
}

bool AccessToSim::getCollision() {
    return this->hasCollision;
}

sim_Command AccessToSim::convertToSimAction(action_t action) {
    sim_Command cmd;

    cmd.i = action.target;
    cmd.x = action.where_To_Act.x;
    cmd.y = action.where_To_Act.y;

    switch(action.type) {
        case no_Command:
            cmd.type = sim_CommandType_NoCommand;
            break;
        case land_On_Top_Of:
            cmd.type = sim_CommandType_LandOnTopOf;
            break;
        case land_In_Front_Of:
            cmd.type = sim_CommandType_LandInFrontOf;
            break;
        case search:
            cmd.type = sim_CommandType_Search;
            break;
    }

    return cmd;
}
