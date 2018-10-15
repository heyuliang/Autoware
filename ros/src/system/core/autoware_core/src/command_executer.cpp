#include <autoware_core/command_executer.h>

command_executer::command_executer()
{

}

command_executer::~command_executer()
{
    
}

void command_executer::execute(std::string command)
{
    std::stringstream ss;
    ss << command;
    boost::property_tree::ptree pt;
    boost::optional<std::string> command_type;
    try
    {
        boost::property_tree::read_json(ss, pt);
        command_type = pt.get_optional<std::string>("command");
    }
    catch(...)
    {
        ROS_ERROR_STREAM("Failed to parse command.");
        return;
    }
    if(command_type == std::string("roslaunch"))
    {
        execute_roslaunch_command_(command);
    }
    return;
}

void command_executer::execute_roslaunch_command_(std::string command)
{
    std::stringstream ss;
    ss << command;
    boost::property_tree::ptree pt;
    boost::optional<std::string> package,launch_filename;
    try
    {
        boost::property_tree::read_json(ss, pt);
        package = pt.get_optional<std::string>("package");
        launch_filename = pt.get_optional<std::string>("launch_filename");
    }
    catch(...)
    {
        ROS_ERROR_STREAM("Failed to parse roslaunch command.");
        return;
    }
    if(package && launch_filename)
    {
        launcher_.launch(package.get(), launch_filename.get());
    }
    else
    {
        ROS_ERROR_STREAM("Failed to get package or launch_filename field");
    }
    return;
}