
#include "lidar_object_visualizer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_object_visualizer");
  VisualizeDetectedObjects app;
  ros::spin();

  return 0;
}
