#include <gtest/gtest.h>
#include "../AI/Robot.h"
#include "../AI/World.h" // trenger jeg denne?

World world = World(0);

class RobotTest: public ::testing::Test { 
public: 
	RobotTest( ) {
	
	} 
	void SetUp( ) {
	}
 };


TEST (RobotTest, addToTimerTest) {
	Robot robot = Robot(); // kan også addes til Setup

	robot.addToTimer(10);
	EXPECT_EQ(robot.getTimeAfterTurn(), 10);
}

TEST (RobotTest, updateTest) {
	Robot robot = Robot();

	robot.update(0, point_Zero, MATH_PI, 1); // test with 21 og 22
	EXPECT_EQ(robot.getOrientation(), 0);
	
	robot.update(0, point_Zero, MATH_PI, 3);
	EXPECT_EQ(robot.getOrientation(), MATH_PI);


}

static point_t point_Updated = {
.x = 2.0,
.y = 3.0,
.z = 1.0,
.travel_Time = 1.0
};

TEST (RobotTest, isMoving) {
	Robot robot = Robot();

	robot.update(0, point_Zero, MATH_PI, 1); // is moving, false even if robot is turning
	EXPECT_EQ(robot.isMoving(),false);

	robot.update(0, point_Zero, MATH_PI, 3);
	EXPECT_EQ(robot.isMoving(),false);

	robot.update(0, point_Updated, MATH_PI, 3);
	EXPECT_EQ(robot.isMoving(),true);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
