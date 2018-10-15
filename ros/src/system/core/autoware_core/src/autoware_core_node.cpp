//headers in ROS
#include <ros/ros.h>

//headers in autoware_core
#include <autoware_core/autoware_core.h>

//headers in httplib
#include <cpp-httplib/httplib.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "autoware_core_node");
  autoware_core core;
  core.run();
  return 0;
}