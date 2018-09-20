#include <emergency_stopper.h>

emergency_stopper::emergency_stopper()
{

}

emergency_stopper::~emergency_stopper()
{

}

void emergency_stopper::publish_state_cmd_()
{
    std_msgs::String cmd;
    cmd.data = "emergency";
    return;
}