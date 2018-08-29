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
  lf.backward_point.x = -1;
  lf.backward_point.y = 0;
  lf.backward_point.z = 0;
  lf.left_width = 2;
  lf.right_width = 3;
  std::vector<geometry_msgs::Point> rectangle = wtg.generateRectangleFromLaneInfo(lf);
  // accesing x in top left point
  EXPECT_EQ(-2, rectangle[0].x);
  // accesing y in top left point
  EXPECT_EQ(2, rectangle[0].y);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
