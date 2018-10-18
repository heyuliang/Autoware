//headers in autoware_core
#include "autoware_core.h"

//headers in httplib
#include <cpp-httplib/httplib.h>

int main(int argc, char *argv[])
{
    autoware_core core;
    core.run();
    return 0;
}
