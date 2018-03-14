#include <gtest/gtest.h>
#include "../AI/Robot.h"
#include "../fuser.h"

World world = World(0);
#define PI 3.14159

class FuserTest: public ::testing::Test {
protected:
    virtual void SetUp() {
    }
};

TEST_F (FuserTest, nearestNeighborTestObviousRobot) {
    // Case 1: Obvious robot
    // ----------------------
    // |                    |
    // |                    |
    // |                    |
    // |      ^             |
    // |      0       1>    |
    // |                    |
    // |                    |
    // ----------------------
    //
    // After 5 seconds
    // ----------------------
    // |                    |
    // |                    |
    // |      ^             |
    // |      x             |
    // |                    |
    // |                    |
    // |                    |
    // ----------------------
    //
    // Should return x = 0

    Robot robot0 = Robot();
    point_t pos = point_zero;
    pos.x = 8;
    pos.y = 10;
    robot0.update(0, pos, PI / 2, 5, true);
    robots_in_memory[0] = robot0;

    Robot robot1 = Robot();
    pos.x = 12;
    pos.y = 10;
    robot1.update(1, pos, 0, 5, true);
    robots_in_memory[1] = robot1;

    Robot robotX = Robot();
    pos.x = 8;
    pos.y = 14;
    robotX.update(-1, pos, PI / 2, 10, true);

    EXPECT_EQ(nearestNeighbor(robotX), 0);
}

TEST_F (FuserTest, nearestNeighborTestNoChangesNoTime) {
    // Case 2: No changes
    // ----------------------
    // |                    |
    // |                    |
    // |                    |
    // |      ^             |
    // |      0       1>    |
    // |                    |
    // |                    |
    // ----------------------
    //
    // After 0 seconds
    // ----------------------
    // |                    |
    // |                    |
    // |                    |
    // |      ^             |
    // |      x       y>    |
    // |                    |
    // |                    |
    // ----------------------
    //
    // Should return x = 0, y = 1

    Robot robot0 = Robot();
    point_t pos = point_zero;
    pos.x = 8;
    pos.y = 10;
    robot0.update(0, pos, PI / 2, 5, true);
    robots_in_memory[0] = robot0;

    Robot robot1 = Robot();
    pos.x = 12;
    pos.y = 10;
    robot1.update(1, pos, 0, 5, true);
    robots_in_memory[1] = robot1;

    Robot robotX = Robot();
    pos.x = 8;
    pos.y = 10;
    robotX.update(-1, pos, PI / 2, 5, true);

    Robot robotY = Robot();
    pos.x = 12;
    pos.y = 10;
    robotY.update(-1, pos, 0, 5, true);

    EXPECT_EQ(nearestNeighbor(robotX), 0);
    EXPECT_EQ(nearestNeighbor(robotY), 1);
}

TEST_F (FuserTest, nearestNeighborTestSameDirection) {
    // Case 2: No changes
    // ----------------------
    // |                    |
    // |         0>         |
    // |                    |
    // |                    |
    // |                    |
    // |         1>         |
    // |                    |
    // ----------------------
    //
    // After 6 seconds
    // ----------------------
    // |                    |
    // |               x>   |
    // |                    |
    // |                    |
    // |                    |
    // |               y>   |
    // |                    |
    // ----------------------
    //
    // Should return x = 0, y = 1

    Robot robot0 = Robot();
    point_t pos = point_zero;
    pos.x = 10;
    pos.y = 15;
    robot0.update(0, pos, 0, 5, true);
    robots_in_memory[0] = robot0;

    Robot robot1 = Robot();
    pos.x = 10;
    pos.y = 5;
    robot1.update(1, pos, 0, 5, true);
    robots_in_memory[1] = robot1;

    Robot robotX = Robot();
    pos.x = 12;
    pos.y = 15;
    robotX.update(-1, pos, 0, 11, true);

    Robot robotY = Robot();
    pos.x = 12;
    pos.y = 5;
    robotY.update(-1, pos, 0, 11, true);

    EXPECT_EQ(nearestNeighbor(robotX), 0);
    EXPECT_EQ(nearestNeighbor(robotY), 1);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
