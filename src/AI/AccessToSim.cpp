#include "AccessToSim.h"
#include "structs.h"
#include <iostream>
#include <array>

AccessToSim::AccessToSim(Observation observation) {
    std::array<point_t, Num_Targets> robots;
    std::array<point_t, Num_Obstacles> obstacles;

    for (int i = 0; i < Num_Robots; i++) {
        Robot robot = observation.getRobot(i);
        point_t pos = robot.getPosition();

        robots[i].x = pos.x;
        robots[i].y = pos.y;
        robots[i].z = robot.getOrientation();
    }

    for (int i = 0; i < Num_Obstacles; i++) {
        Robot obstacle = observation.getObstacle(i);
        point_t pos = obstacle.getPosition();

        obstacles[i].x = pos.x;
        obstacles[i].y = pos.y;
        obstacles[i].z = obstacle.getOrientation();
    }

    sim_State state = sim_init_state(observation.getTimeStamp(), robots, obstacles);

    this->state = state;
}

AccessToSim* AccessToSim::step() {
    sim_Command cmd;
    cmd.type = sim_CommandType_NoCommand;

    this->state = sim_tick(this->state, cmd);

    return this;
}

sim_State AccessToSim::getState() {
    return this->state;
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

bool AccessToSim::getCollision() {
    return this->hasCollision;
}
