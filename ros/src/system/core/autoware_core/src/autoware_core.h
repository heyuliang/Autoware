#ifndef AUTOWARE_CORE_H_INCLUDED
#define AUTOWARE_CORE_H_INCLUDED

//headers in STL
#include <thread>
#include <memory>

//headers in httplib
#include <cpp-httplib/httplib.h>

//heaaders in Autoware
#include "command_executer.h"

class autoware_core
{
public:
    autoware_core();
    ~autoware_core();
    void run();
private:
    std::string request_to_string_(httplib::Request req);
    void run_server_();
    httplib::Server server_;
    int port_;
    command_executer command_executer_;
    std::function<void (const httplib::Request&, httplib::Response&)> execute_roslaunch_func_;
    std::shared_ptr<std::thread> server_thread_ptr_;
};

#endif  //AUTOWARE_CORE_H_INCLUDED
