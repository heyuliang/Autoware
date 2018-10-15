#include <autoware_core/node_launcher.h>

node_launcher::node_launcher()
{

}

node_launcher::~node_launcher()
{

}

void node_launcher::launch(std::string package,std::string launch_filename)
{
    launch_info info(package,launch_filename);
    return;
}

void node_launcher::execute_(launch_info info)
{
    mtx_.lock();
    launched_info_.push_back(info);
    mtx_.unlock();

    mtx_.lock();
    /*
    for(auto itr=launched_info_.begin() ; itr!=launched_info_.end(); itr++)
    {
        if(itr->package == info.package && itr->launch_filename == info.launch_filename)
        {
            //launched_info_.erase(itr);
            break;
        }
        /*
        if(launched_info_[i].package == info.package && launched_info_[i].launch_filename == info.launch_filename)
        {
            launched_info_.erase(launched_info_.begin());
            //launched_info_.erase(launched_info_.begin());
            //launched_info_.begin() + (unsigned int)i;
            //launched_info_.erase(launched_info_.begin() + (unsigned int)(i));
            break;
        }
    }
    */
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