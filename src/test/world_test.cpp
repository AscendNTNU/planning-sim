#include <gtest/gtest.h>
#include "../AI/World.cpp"


class testFixture: public ::testing::Test { 
public: 
	testFixture( ) {
	
	} 
	void SetUp( ) {

	}
 };

TEST (testFixture, ConstructorTest) {
	World world = World(3.14);
	EXPECT_EQ(world.getOrigin().x, point_Zero.x);
	EXPECT_EQ(world.getOrigin().y, point_Zero.y);
	EXPECT_NEAR(world.getOrientation(), 3.14, 0.01);
	EXPECT_EQ(world.getBounds().x_Max, 20);
	EXPECT_EQ(world.getBounds().y_Max, 20);
}

TEST (testFixture, getGridValue) {
	World world = World(0);
	// EXPECT_EQ(world.getGridValue(0, 0), )
	// EXPECT_EQ(world.getGridValue(10, 10), )
	// EXPECT_EQ(world.getGridValue(10, 20), )
	// EXPECT_EQ(world.getGridValue(10, 20), )
	// EXPECT_EQ(world.getGridValue(10, 0), )

	//Test that some points have higher values than others
	EXPECT_GT(world.getGridValue(10, 20),world.getGridValue(10, 10));
	EXPECT_GT(world.getGridValue(10, 20),world.getGridValue(20, 20));
	EXPECT_GT(world.getGridValue(10, 20),world.getGridValue(0, 20));

	EXPECT_GT(world.getGridValue(10, 10),world.getGridValue(0, 10));
	EXPECT_GT(world.getGridValue(10, 10),world.getGridValue(10, 0));
	EXPECT_GT(world.getGridValue(10, 10),world.getGridValue(20, 10));

	EXPECT_GT(world.getGridValue(20, 10),world.getGridValue(20, 0));
	EXPECT_GT(world.getGridValue(0, 10),world.getGridValue(0, 0));

}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}