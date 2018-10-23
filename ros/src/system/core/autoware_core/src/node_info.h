#ifndef NODE_INFO_H_INCLUDED
#define NODE_INFO_H_INCLUDED

struct node_info
{
    const std::string packaage;
    const std::string type;
    const std::string name;
    node_info(std::string packaage_str, std::string type_str, std::string name_str) : packaage(packaage_str), type(type_str), name(name_str)
    {

    }
};
#endif  //NODE_INFO_H_INCLUDED