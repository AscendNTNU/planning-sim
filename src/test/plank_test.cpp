#include <gtest/gtest.h>
#include "../AI/Plank.cpp"
#include "../AI/World.cpp"

World* world = new World(0);

class testFixture: public ::testing::Test { 
public: 
	testFixture( ) {
	
	} 
	void SetUp( ) {
		world->startTimer();
	}
 };

TEST (textFixture, EmptyConstructorTest) {
	Plank* plank = new Plank();
	EXPECT_EQ(plank->getPoint(1).x, point_Zero.x);
	EXPECT_EQ(plank->getPoint(1).y, point_Zero.y);
}

TEST (textFixture, ConstructorTest) {
	//This technically also tests the updatePlank function
	point_t point = point_Zero;
	point.x = 10;
	point.y = 10;

	float angle = 3.14;
	int time_after_turn_start = 0;
	int number_of_iterations = 10;

	Plank* plank = new Plank(point, angle, time_after_turn_start, number_of_iterations);
	EXPECT_EQ(plank->getPoint(1).x, point.x);
	EXPECT_EQ(plank->getPoint(1).y, point.y);
	EXPECT_NEAR(plank->getPoint(2).x, point.x + 5.94, 0.1);
	EXPECT_NEAR(plank->getPoint(2).y, point.y, 0.1);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}