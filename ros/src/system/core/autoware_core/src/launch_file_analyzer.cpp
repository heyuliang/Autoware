#include "launch_file_analyzer.h"

launch_file_analyzer::launch_file_analyzer()
{

}

launch_file_analyzer::~launch_file_analyzer()
{

}

boost::optional<std::vector<node_info> > launch_file_analyzer::read(std::string launch_file_path)
{
    namespace fs = boost::filesystem;
    fs::path path(launch_file_path);
    boost::system::error_code error;
    const bool result = fs::exists(path, error);
    if (!result || error)
    {
        std::cout << "could not find " << launch_file_path << std::endl;
        return boost::none;
    }
    std::vector<node_info> node_info_data;
    return read_xml_(launch_file_path,node_info_data);
}

boost::optional<std::vector<node_info> > launch_file_analyzer::read_xml_(std::string launch_file_path,std::vector<node_info> node_info_data)
{
    std::vector<node_info> ret;
    boost::property_tree::ptree pt;
    try
    {
        boost::property_tree::xml_parser::read_xml(launch_file_path, pt);
    }
    catch(...)
    {
        std::cout << "failed to read " << launch_file_path << std::endl;
        return boost::none;
    }
    for (auto it : pt.get_child("launch"))
    {
        std::string package_name_str;
        std::string node_name_str;
        std::string type_name_str;
        if(it.first == "node")
        {
            if(auto package_name = it.second.get_optional<std::string>("<xmlattr>.pkg"))
            {
                if(!package_name)
                {
                    continue;
                }
                package_name_str = package_name.get();
            }
            if(auto node_name = it.second.get_optional<std::string>("<xmlattr>.name"))
            {
                if(!node_name)
                {
                    continue;
                }
                node_name_str = node_name.get();
            }
            if(auto type_name = it.second.get_optional<std::string>("<xmlattr>.type"))
            {
                if(!type_name)
                {
                    continue;
                }
                type_name_str = type_name.get();
            }
        }
        node_info info(package_name_str,type_name_str,node_name_str);
        ret.push_back(info);
    }
    return ret;
}