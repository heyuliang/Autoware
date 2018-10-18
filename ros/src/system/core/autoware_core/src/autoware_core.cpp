#include "autoware_core.h"

autoware_core::autoware_core()
{
    port_ = 8080;
}

autoware_core::~autoware_core()
{

}

void autoware_core::run()
{
    execute_roslaunch_func_ = std::bind(&command_executer::execute_roslaunch_command, &command_executer_, std::placeholders::_1, std::placeholders::_2);
    server_.Post("/roslaunch", execute_roslaunch_func_);
    server_.listen("localhost", port_);
    return;
}
