//headers in autoware_core
#include "launch_file_analyzer.h"

//headers in ROS
#include <ros/package.h>

int main(int argc, char *argv[])
{
    std::string path = ros::package::getPath("autoware_core")+std::string("/launch/autoware_core.launch");
    launch_file_analyzer analyzer;
    analyzer.read(path);
}