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
	point_t position = drone->getPosition();
	EXPECT_EQ(position.x, 0);
	EXPECT_EQ(position.y, 0);
}

TEST (DroneTest, DistanceToPointTest) {
	Drone* drone = new Drone();
	point_t point = point_Zero;
	point.x = 3;
	point.y = 4;
	EXPECT_EQ(drone->getDistanceToPoint(point), 5);

	point.x = -3;
	point.y = 4;
	EXPECT_EQ(drone->getDistanceToPoint(point), 5);

	point.x = 0;
	point.y = 0;
	EXPECT_EQ(drone->getDistanceToPoint(point), 0);

	point.x = 5;
	point.y = -5;
	EXPECT_NEAR(drone->getDistanceToPoint(point), 7.07f, 0.01f);

	point.x = 123456789;
	point.y = 987654321;
	EXPECT_NEAR(drone->getDistanceToPoint(point), 995340462.62581f, 0.0001f);
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
