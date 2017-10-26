#include <gtest/gtest.h>
#include "../AI/Plank.cpp"
#include "../AI/World.cpp"

World world = World(0);

class PlankTest: public ::testing::Test { 
public: 

	void SetUp( ) {
		world.startTimer();
	}
 };

TEST (PlankTest, EmptyConstructorTest) {
	Plank plank = Plank();
	EXPECT_EQ(plank.getPoint(1).point.x, point_Zero.x);
	EXPECT_EQ(plank.getPoint(1).point.y, point_Zero.y);
}

TEST (PlankTest, UpdatePlankTest) {
	point_t point = point_Zero;
	point.x = 10;
	point.y = 10;

	float angle = 3.14;
	int time_after_turn_start = 0;
	int number_of_iterations = 10;

	//5.94 meters is approximately how far a ground robot drives in 18 seconds
	Plank plank = Plank(point, angle, time_after_turn_start);
	EXPECT_EQ(plank.getPoint(11).point.x, point.x);
	EXPECT_EQ(plank.getPoint(11).point.y, point.y);
	EXPECT_NEAR(plank.getPoint(0).point.x, point.x + 5.94, 0.1);
	EXPECT_NEAR(plank.getPoint(0).point.y, point.y, 0.1);

	time_after_turn_start = 11; 
	plank.updatePlank(point, angle, time_after_turn_start);
	EXPECT_NEAR(plank.getPoint(11).point.x, point.x - 5.94/2, 0.1);
	EXPECT_NEAR(plank.getPoint(11).point.y, point.y, 0.1);
	EXPECT_NEAR(plank.getPoint(0).point.x, point.x + 5.94/2, 0.1);
	EXPECT_NEAR(plank.getPoint(0).point.y, point.y, 0.1);
}

TEST(PlankTest, PointOutsideOfPlank){
	point_t point = point_Zero;
	point.x = 10;
	point.y = 10;

	float angle = 3.14;
	int time_after_turn_start = 11;
	int number_of_iterations = 10;

	Plank plank = Plank(point, angle, time_after_turn_start);

	point.x = 5;
	EXPECT_TRUE(plank.pointIsOutsideOfPlank(point));
	point.x = 7.5;
	EXPECT_FALSE(plank.pointIsOutsideOfPlank(point));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}