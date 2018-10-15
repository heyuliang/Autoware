#ifndef NODE_LAUNCHER_H_INCLUDED
#define NODE_LAUNCHER_H_INCLUDED

//headers in STL
#include <string>
#include <vector>
#include <mutex>
#include <stdlib.h>
#include <thread>

class launch_info
{
public:
    launch_info(std::string package, std::string launch_filename)
    {
        package_ = package;
        launch_filename_ = launch_filename;
    }
    std::string get_package(){return package_;};
    std::string get_launch_filename(){return launch_filename_;};
private:
    std::string package_;
    std::string launch_filename_;

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