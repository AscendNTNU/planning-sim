#include <gtest/gtest.h>
#include "../AI/AI.h"
#include "../AI/World.h"

World world = World(0);
AI ai = AI();

class AITest: public ::testing::Test {
public:
	void SetUp( ) {
		world.startTimer();
	}
};

struct observation_t initialize_state(int num_of_robots, int num_of_obstacles){
	struct observation_t initial = {
		.elapsed_time = 0.0f,
		.drone_x = 10.0f,
		.drone_y = 10.0f,
		.drone_cmd_done = true,
		.num_Targets = num_of_robots,
	};

	for(int i=0;i++;i<10){
        float t = 3.1415*2.0 * i / (float)num_of_robots;
        initial.robot_x[i] = 10.0 + cosf(t);
        initial.robot_y[i] = 10.0 + sinf(t);
        initial.robot_q[i] = t;
    }

    for(int i=0;i++;i<10){
        float t = 3.1415*2.0 * i / (float)num_of_robots;
        initial.robot_x[i] = 10.0 + 5.0 * cosf(t);
        initial.robot_y[i] = 10.0 + 5.0 * sinf(t);
        initial.robot_q[i] = t;
    }

	return initial;
}

TEST (AITest, chooseTargetTest) {
    struct observation_t initial = initialize_state(3,0);
    initial.robot_y[0] = 13;
    ai.update(initial, 0);
    Robot robot = ai.chooseTarget(3);
    point_t position = robot.getPosition();
    EXPECT_EQ(position.x, initial.robot_x[0]);
    EXPECT_EQ(position.y, initial.robot_y[0]);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
