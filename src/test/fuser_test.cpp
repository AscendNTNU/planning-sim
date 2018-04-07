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

Robot createRobot(float x, float y, float q, float elapsed_time, int index) {
    Robot robot;
    point_t pos = point_zero;
    pos.x = x;
    pos.y = y;
    robot.update(index, pos, q, elapsed_time, true);
    return robot;
}

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

    robots_in_memory[0] = createRobot(8, 10, PI/2, 5, 0);
    robots_in_memory[1] = createRobot(12, 10, 0, 5, 1);

    Robot robotX = createRobot(8, 14, PI/2, 10, -1);

    std::set<int> used_indices;
    EXPECT_EQ(nearestNeighbor(robotX, used_indices), 0);
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

    robots_in_memory[0] = createRobot(8, 10, PI/2, 5, 0);
    robots_in_memory[1] = createRobot(12, 10, 0, 5, 1);

    Robot robotX = createRobot(8, 10, PI/2, 5, -1);
    Robot robotY = createRobot(12, 10, 0, 5, -1);

    std::set<int> used_indices;
    EXPECT_EQ(nearestNeighbor(robotX, used_indices), 0);
    used_indices.insert(0);
    EXPECT_EQ(nearestNeighbor(robotY, used_indices), 1);
}

TEST_F (FuserTest, nearestNeighborTestSameDirection) {
    // Case 3: Parallell robots in same direction
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

    robots_in_memory[0] = createRobot(10, 15, 0, 5, 0);
    robots_in_memory[1] = createRobot(10, 5, 0, 5, 1);

    Robot robotX = createRobot(12, 15, 0, 11, -1);
    Robot robotY = createRobot(12, 5, 0, 11, -1);

    std::set<int> used_indices;
    EXPECT_EQ(nearestNeighbor(robotX, used_indices), 0);
    used_indices.insert(0);
    EXPECT_EQ(nearestNeighbor(robotY, used_indices), 1);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
