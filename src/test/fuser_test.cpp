#include <gtest/gtest.h>
#include "../AI/Robot.h"
#include "../fuser.h"

World world = World(0);

class FuserTest: public ::testing::Test {
protected:
    virtual void SetUp() {
    }
};

TEST_F (FuserTest, nearestNeighborTest) {
    Robot robot;
    std::cout << "works!!" << robots_in_memory.size() << std::endl;
    EXPECT_EQ(1, 1);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
