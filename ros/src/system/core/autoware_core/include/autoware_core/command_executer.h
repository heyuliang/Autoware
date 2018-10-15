#ifndef COMMAND_EXECUTER_H_INCLUDED
#define COMMAND_EXECUTER_H_INCLUDED

//headers in autoware_core
#include <autoware_core/node_launcher.h>

//headers in boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

class command_executer
{
public:
    command_executer();
    ~command_executer();
    //parse();
private:
    node_launcher launcher_;
};

#endif  //COMMAND_EXECUTER_H_INCLUDED