#include "autoware_core.h"

namespace autoware_core
{

HttpServer::HttpServer(const std::__cxx11::string host, int port)
{
    host_ = host;
    port_ = port;
}

HttpServer::~HttpServer()
{

}

void HttpServer::run()
{
    //execute_roslaunch_func_ = std::bind(&command_executer::execute_roslaunch_command, &command_executer_, std::placeholders::_1, std::placeholders::_2);
    //server_.Post("/roslaunch", execute_roslaunch_func_);

    server_.Get("/test", [](const httplib::Request& req, httplib::Response& res)
    {
        res.set_content("Hello World!", "text/plain");
    });

    printf("Server is started at %s:%d\n", host_.c_str(), port_);
    server_.listen(host_.c_str(), port_);
}

}
