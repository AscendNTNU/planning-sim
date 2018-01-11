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

void AccessToSim::step() {

    sim_Command cmd;
    cmd.type = sim_CommandType_NoCommand;

    this->state = sim_tick(this->state, cmd);
}
