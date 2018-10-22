#ifndef AUTOWARE_CORE_HTTP_SERVER_H_INCLUDED
#define AUTOWARE_CORE_HTTP_SERVER_H_INCLUDED

#include <string>
#include <cpp-httplib/httplib.h>

namespace autoware_core
{

class HttpServer
{
    public:

        HttpServer(const std::string host, int port);
        ~HttpServer();

        void run();

    private:

        std::string request_to_string_(httplib::Request req);
        void run_server_();

        httplib::Server server_;
        int port_;
        std::string host_;
};

}

#endif  // AUTOWARE_CORE_HTTP_SERVER_H_INCLUDED
