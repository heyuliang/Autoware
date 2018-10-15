#include <autoware_core/command_executer.h>

command_executer::command_executer()
{

}

command_executer::~command_executer()
{
    
}

void command_executer::execute_roslaunch_command(const httplib::Request& req, httplib::Response& res)
{
    std::string command  = req.body;
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
        pt.put("response.success", false);
        pt.put("response.description", "Failed to parse roslaunch command.");
        return;
    }
    if(package && launch_filename)
    {
        launcher_.launch(package.get(), launch_filename.get());
        pt.put("response.success", true);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to get package or launch_filename field.");
        pt.put("response.success", false);
        pt.put("response.description", "Failed to get package or launch_filename field.");
    }
    write_json(ss, pt);
    res.set_content(ss.str(),"text/plain");
    //res.body = ss.str();
    return;
}