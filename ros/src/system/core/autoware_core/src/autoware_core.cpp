#include <autoware_core/autoware_core.h>

autoware_core::autoware_core()
{
    nh_.param<int>(ros::this_node::getName()+"/port", port_, 8000);
}

autoware_core::~autoware_core()
{

}

void autoware_core::run()
{
    server_.listen("localhost", 8080);
}