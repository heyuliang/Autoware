#ifndef EMERGENCY_STOPPER_H_INCLUDED
#define EMERGENCY_STOPPER_H_INCLUDED

//headers in diag_lib
#include <diag_msgs/diag.h>

//headers in ROS
#include <ros/ros.h>
#include <std_msgs/String.h>

class emergency_stopper
{
public:
    emergency_stopper();
    ~emergency_stopper();
private:
    void publish_state_cmd_();
    ros::NodeHandle _nh;
    ros::Subscriber _diag_sub;
    ros::Publisher _cmd_pub;
};
#endif  //EMERGENCY_STOPPER_H_INCLUDED