#include <gtest/gtest.h>
#include "../AI/Drone.h"
#include "../AI/World.cpp"

World* world = new World(0);

class DroneTest: public ::testing::Test {
public:
	DroneTest( ) {
	
	}
	void SetUp( ) {
		world->startTimer();
	}
};

TEST (DroneTest, EmptyConstructorTest) {
	Drone* drone = new Drone();
	point_t pos = drone->getPosition();
	EXPECT_EQ(pos.x, 0);
	EXPECT_EQ(pos.y, 0);
}

TEST (DroneTest, DistanceToPointTest) {
	// Lag en test her...
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
