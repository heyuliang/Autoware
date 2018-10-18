#include "node_launcher.h"


//headers in boost
#include <boost/thread.hpp>

LaunchElement::LaunchElement( std::string filename )
{
    filename_ = filename;
}

std::string LaunchElement::execute()
{
    std::string error;

    // parse XML

    // create node
    // ros->lock();
    // check node duplication
    // add to launch collection
    // add to node collection
    // ros->unlock();

    //std::string command = "roslaunch " + filename_;
    //system( command.c_str() );

    // ros->lock();
    // remove from node collection
    // remove from launch collection
    // ros->unlock();

    return error;
}

node_launcher::node_launcher()
{

}

node_launcher::~node_launcher()
{

}

void node_launcher::launch(std::string package,std::string launch_filename)
{
    launch_info info(package,launch_filename);
    boost::thread exec_thread(&node_launcher::execute_, this, info);
}

void node_launcher::execute_(launch_info info)
{
    mtx_.lock();
    launched_info_.push_back(info);
    mtx_.unlock();

    std::string cmd = "roslaunch " + info.get_package() + " " + info.get_launch_filename();
    int ret = system(cmd.c_str());

    mtx_.lock();
    std::vector<launch_info> new_launched_info;
    for(auto itr=launched_info_.begin() ; itr!=launched_info_.end(); itr++)
    {
        if(itr->get_package() != info.get_package() || itr->get_launch_filename() != info.get_launch_filename())
        {
            new_launched_info.push_back(*itr);
        }
    }
    launched_info_ = new_launched_info;
    mtx_.unlock();
    return;
}

std::vector<launch_info> node_launcher::get_launched_info()
{
    mtx_.lock();
    std::vector<launch_info> ret = launched_info_;
    mtx_.unlock();
    return ret;
}
