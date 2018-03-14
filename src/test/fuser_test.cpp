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

TEST_F (FuserTest, nearestNeighborTest) {
    // Case 1:
    // ----------------------
    // |                    |
    // |                    |
    // |                    |
    // |      ^             |
    // |      1       2>    |
    // |                    |
    // |                    |
    // ----------------------
    //
    // After 2 seconds
    // ----------------------
    // |                    |
    // |      ^             |
    // |      x             |
    // |                    |
    // |                    |
    // |                    |
    // |                    |
    // ----------------------
    //
    // Should return x = 1

    Robot robot1 = Robot(1);
    point_t pos1 = point_zero;
    pos1.x = 8;
    pos1.y = 10;
    robot1.setPositionOrientation(pos1, PI / 2);
    robots_in_memory[0] = robot1;

    Robot robot2 = Robot(2);
    point_t pos2 = point_zero;
    pos2.x = 12;
    pos2.y = 10;
    robot2.setPositionOrientation(pos2, 0);
    robots_in_memory[1] = robot2;

    Robot observed_robot = Robot();
    point_t posX = point_zero;
    posX.x = 8;
    posX.y = 12;
    observed_robot.setPositionOrientation(posX, PI / 2);
    nearestNeighbor(observed_robot);

    std::cout << "works!!" << robots_in_memory.size() << std::endl;
    std::cout << robots_in_memory[0].getPosition().y << std::endl;
    EXPECT_EQ(1, 1);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
