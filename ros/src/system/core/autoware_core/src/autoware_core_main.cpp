//headers in autoware_core
#include "autoware_core.h"

int main(int argc, char *argv[])
{
    using namespace autoware_core;

    HttpServer http_server("localhost", 8080);
    http_server.run();
}
