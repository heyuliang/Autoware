#ifndef AUTOWARE_CORE_H_INCLUDED
#define AUTOWARE_CORE_H_INCLUDED

//headers in STL
#include <thread>

//headers in ROS
#include <ros/ros.h>

//headers in httplib
#include <cpp-httplib/httplib.h>

//heaaders in Autoware
#include <autoware_core/command_executer.h>

class autoware_core
{
public:
    autoware_core();
    ~autoware_core();
    void run();
private:
    std::string request_to_string_(httplib::Request req);
    void run_server_();
    ros::NodeHandle nh_;
    httplib::Server server_;
    int port_;
    command_executer command_executer_;
    std::function<void (const httplib::Request&, httplib::Response&)> execute_roslaunch_func_;
};

#endif  //AUTOWARE_CORE_H_INCLUDED