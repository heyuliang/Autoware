#include <emergency_stopper.h>

emergency_stopper::emergency_stopper()
{
    _diag_sub = _nh.subscribe("/watchdog_node/diag/all", 1, &emergency_stopper::_diag_callback, this);
    _cmd_pub = _nh.advertise<std_msgs::String>("/state_cmd", 1);
}

emergency_stopper::~emergency_stopper()
{

}

void emergency_stopper::_publish_state_cmd()
{
    std_msgs::String cmd;
    cmd.data = "emergency";
    _cmd_pub.publish(cmd);
    return;
}

void emergency_stopper::_diag_callback(diag_msgs::diag msg)
{
    boost::optional<diag_msgs::diag_node_errors> error = _diag_filter.filter(msg,"/decision_maker");
    if(error)
    {
        _publish_state_cmd();
    }
    return;
}