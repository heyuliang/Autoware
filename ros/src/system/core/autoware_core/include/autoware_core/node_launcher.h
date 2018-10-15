#ifndef NODE_LAUNCHER_H_INCLUDED
#define NODE_LAUNCHER_H_INCLUDED

//headers in STL
#include <string>
#include <vector>
#include <mutex>

//headers in boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

struct launch_info
{
    const std::string package;
    const std::string launch_filename;
    launch_info(std::string package_, std::string launch_filename_) : package(package_), launch_filename(launch_filename_)
    {

    }
};

class node_launcher
{
public:
    node_launcher();
    ~node_launcher();
    void launch(std::string package,std::string launch_filename);
    std::vector<launch_info> get_launched_info();
private:
    std::mutex mtx_;
    void execute_(launch_info info);
    std::vector<launch_info> launched_info_;
};

#endif  //NODE_LAUNCHER_H_INCLUDED