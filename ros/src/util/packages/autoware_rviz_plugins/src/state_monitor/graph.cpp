#include "graph.hpp"

#include <yaml-cpp/yaml.h>
#include <iostream>

namespace autoware_rviz_plugins {
namespace state_monitor {

StateNode::StateNode(const std::string &graph, const std::string &state, const std::string &parent)
{
    depth_  = -1;
    graph_  = graph;
    state_  = state;
    parent_ = parent;
}

int StateNode::updateDepth(std::map<std::string, StateNode>& nodes)
{
    if(0 <= depth_)
    {
        return depth_;
    }
    if(nodes.count(parent_) == 0)
    {
        return (depth_ = 0);
    }
    return (depth_ = nodes[parent_].updateDepth(nodes) + 1);
}



StateGraph::StateGraph()
{
    clear();
}

bool StateGraph::isOK()
{
    return ok_;
}

bool StateGraph::load(const std::string& graph, const std::string &filename)
{
    try
    {
        YAML::Node yaml_root = YAML::LoadFile(filename);
        for(const YAML::Node& node : yaml_root["autoware_states"])
        {
            const std::string state  = node["StateName"].as<std::string>();
            const std::string parent = node["Parent"].as<std::string>();
            nodes_[state] = StateNode(graph, state, parent);
        }
        height_[graph] = 0;
    }
    catch( ... )
    {
        return false;
    }
    return true;
}

void StateGraph::clear()
{
    ok_ = false;
    height_.clear();
    nodes_.clear();
}

void StateGraph::construct()
{
    for(auto& it : nodes_)
    {
        std::string graph = it.second.graph_;
        height_[graph] = std::max(height_[graph], it.second.updateDepth(nodes_));
    }

    for(auto a : height_)
    {
        std::cout << a.first << " " << a.second << std::endl;
    }

    ok_ = true;
}

bool StateGraph::getState(std::string state, StateNode& node)
{
    if(!nodes_.count(state))
    {
        return false;
    }

    node = nodes_[state];
    return true;
}

std::map<std::string, int> StateGraph::getHeight()
{
    return height_;
}


}}
