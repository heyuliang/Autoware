// Bring in my package's API, which is what I'm testing
// #include "nodes/wayarea2grid/wayarea2grid.h"
#include "wayarea2grid.h"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(Wayarea2Grid, generateRectangleFromLaneInfo)
{
  object_map::WayareaToGrid wtg;
  LaneInfo lf;
  lf.point.x = 0;
  lf.point.y = 1;
  lf.point.z = 0;
  lf.forward_point.x = 0;
  lf.forward_point.y = 3;
  lf.forward_point.z = 0;
  lf.backward_point.x = 0;
  lf.backward_point.y = -1;
  lf.backward_point.z = 0;
  lf.left_width = 2;
  lf.right_width = 2;
  std::vector<geometry_msgs::Point> rectangle = wtg.generateRectangleFromLaneInfo(lf);
  // accesing x in top left point
  EXPECT_EQ(-2, rectangle[0].x);
  // accesing y in top left point
  EXPECT_EQ(4, rectangle[0].y);
}

TEST(Wayarea2Grid, generate45InclinedRectangleFromLaneInfo)
{
  object_map::WayareaToGrid wtg;
  LaneInfo lf;
  lf.point.x = 1;
  lf.point.y = 1;
  lf.point.z = 0;
  lf.forward_point.x = 3;
  lf.forward_point.y = 3;
  lf.forward_point.z = 0;
  lf.backward_point.x = -1;
  lf.backward_point.y = -1;
  lf.backward_point.z = 0;
  lf.left_width = 1.41421356*2;
  lf.right_width = 1.41421356;
  std::vector<geometry_msgs::Point> rectangle = wtg.generateRectangleFromLaneInfo(lf);
  // accesing top left point
  EXPECT_NEAR(2., rectangle[0].x, 0.001);
  EXPECT_NEAR(6., rectangle[0].y, 0.001);

  //accesing bottom left point
  EXPECT_NEAR(-4., rectangle[3].x, 0.001);
  EXPECT_NEAR(0., rectangle[3].y, 0.001);
}

TEST(Wayarea2Grid, generate60InclinedRectangleFromLaneInfo)
{
  object_map::WayareaToGrid wtg;
  LaneInfo lf;
  lf.point.x = 1;
  lf.point.y = 1.7320508;
  lf.point.z = 0;
  lf.forward_point.x = 2;
  lf.forward_point.y = 1.7320508*2;
  lf.forward_point.z = 0;
  lf.backward_point.x = 0;
  lf.backward_point.y = 0;
  lf.backward_point.z = 0;
  lf.left_width  = 2/1.7320508;
  lf.right_width = 2/1.7320508;
  std::vector<geometry_msgs::Point> rectangle = wtg.generateRectangleFromLaneInfo(lf);
  // accesing top left point
  EXPECT_NEAR(1.5, rectangle[0].x, 0.001);
  EXPECT_NEAR(4.90747, rectangle[0].y, 0.001);
}
// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
