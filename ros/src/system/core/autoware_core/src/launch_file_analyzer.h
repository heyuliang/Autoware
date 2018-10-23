#ifndef LAUNCH_FILE_ANALYZER_H_INCLUDED
#define LAUNCH_FILE_ANALYZER_H_INCLUDED

//headers in boost
#define BOOST_PROPERTY_TREE_RAPIDXML_STATIC_POOL_SIZE 512
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

//headers in STL
#include <iostream>
#include <vector>
#include <regex>

//headers in this package
#include "node_info.h"

//headers in ROS
#include <ros/package.h>

class launch_file_analyzer
{
public:
    launch_file_analyzer();
    ~launch_file_analyzer();
    boost::optional<std::vector<node_info> > read(std::string launch_file_path);
private:
    boost::optional<std::vector<node_info> > read_xml_(std::string launch_file_path,std::vector<node_info> node_info_data);
};
#endif  //LAUNCH_FILE_ANALYZER_H_INCLUDED