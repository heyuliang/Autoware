#ifndef AUTOWARE_CORE_H_INCLUDED
#define AUTOWARE_CORE_H_INCLUDED

//headers in STL
#include <thread>

//headers in ROS
#include <ros/ros.h>

//headers in httplib
#include <cpp-httplib/httplib.h>

class autoware_core
{
public:
    autoware_core();
    ~autoware_core();
    void run();
private:
    void run_server_();
    ros::NodeHandle nh_;
    httplib::Server server_;
    int port_;
};

#endif  //AUTOWARE_CORE_H_INCLUDED