#include <gtest/gtest.h>
#include "../AI/AI.h"
#include "../AI/World.h"

World world = World(0);

float PI = 3.1415;

class AITest: public ::testing::Test {
protected:
	virtual void SetUp( ) {
        ai = AI();
	}
    AI ai;

};

struct observation_t initializeObservation(int num_of_robots, int num_of_obstacles){
	struct observation_t initial = {
		.elapsed_time = 23.0f,
		.drone_x = 10.0f,
		.drone_y = 10.0f,
		.drone_cmd_done = true,
		.num_Targets = num_of_robots
	};

	for(int i=0;i<num_of_robots;i++){
        float t = PI*2.0 * i / (float)num_of_robots;
        initial.robot_x[i] = 10.0 + cosf(t);
        initial.robot_y[i] = 10.0 + sinf(t);
        initial.robot_q[i] = t;
    }

    for(int i=0;i<num_of_obstacles;i++){
        float t = PI*2.0 * i / (float)num_of_obstacles;
        initial.obstacle_x[i] = 10.0 + 5.0 * cosf(t);
        initial.obstacle_y[i] = 10.0 + 5.0 * sinf(t);
        initial.obstacle_q[i] = t;
    }

	return initial;
}

// TEST_F (AITest, chooseTargetTest) {
//     struct observation_t initial = initializeObservation(2,0);
//     initial.robot_y[0] = 13;
//     initial.robot_y[1] = 9;

//     initial.robot_q[0] = PI/2.0;
//     initial.robot_q[1] = PI/2.0;

//     ai.update(initial, 0);
//     Robot robot = ai.chooseTarget(2);
//     point_t position = robot.getPosition();
//     EXPECT_EQ(position.x, initial.robot_x[0]);
//     EXPECT_EQ(position.y, initial.robot_y[0]);
// }

// TEST_F (AITest, chooseActionTest) { 
//     struct observation_t initial = initializeObservation(1,0);
//     initial.robot_y[0] = 13;
//     initial.robot_q[0] = PI/2.0;
//     initial.drone_y = 13;
//     initial.drone_x = initial.robot_x[0];

//     ai.update(initial, 0);

//     Robot robot = ai.state.getRobot(0);
//     action_t action = ai.chooseAction(robot);
//     EXPECT_GE(action.where_To_Act.y, 18);
// }

// TEST_F (AITest, getBestActionAtPositionTest) {
//     struct observation_t initial = initializeObservation(1,0);
//     initial.robot_y[0] = 13;
//     initial.robot_q[0] = PI;

//     ai.update(initial, 0);

//     Robot robot = ai.state.getRobot(0);

//     struct point_t pos = {
//         .x = initial.robot_x[0],
//         .y = robot.current_Plank.getPoint(11).point.y
//     };

//     struct plank_point_t position = {
//         .point = pos,
//         .is_ahead = true,
//         .time_till_first_arrival = 4,
//         .time_since_start_turn = robot.current_Plank.getPoint(11).time_since_start_turn
//     };

//     action_t action = ai.getBestActionAtPosition(robot.getOrientation(), position);

//     EXPECT_EQ(robot.current_Plank.getPoint(0).point.y,14);
//     // EXPECT_EQ(land_In_Front_Of, action.type); 
// }

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
