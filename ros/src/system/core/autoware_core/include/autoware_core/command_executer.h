#ifndef COMMAND_EXECUTER_H_INCLUDED
#define COMMAND_EXECUTER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>

//headers in autoware_core
#include <autoware_core/node_launcher.h>

//headers in httplib
#include <cpp-httplib/httplib.h>

//headers in boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

class command_executer
{
public:
    command_executer();
    ~command_executer();
    void execute_roslaunch_command(const httplib::Request& req, httplib::Response& res);
private:
    //parameters for roslaunch command
    node_launcher launcher_;
};

#endif  //COMMAND_EXECUTER_H_INCLUDED