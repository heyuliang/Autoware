// headers in decision_maker
#include <emergency_stopper.h>

// headers in ROS
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "emergency_stopper");
  emergency_stopper stopper;
  ros::spin();
  return 0;
}
