#include "graph.hpp"

#include <yaml-cpp/yaml.h>
#include <sstream>

namespace autoware_rviz_plugins {
namespace state_monitor {

StateFrame::StateFrame()
{
    duration = 0;
    empty = true;
    name.clear();
}

StateNode::StateNode(const std::string &group, const std::string &state, const std::string &parent)
{
    this->depth  = -1;
    this->group  = group;
    this->state  = state;
    this->parent = parent;
}

int StateNode::updateDepth(std::map<std::string, StateNode>& nodes)
{
    if(0 <= depth)
    {
        return depth;
    }

    if(nodes.count(parent) == 0)
    {
        depth = 0;
    }
    else
    {
        depth = nodes[parent].updateDepth(nodes) + 1;
    }

    return depth;
}



StateGraph::StateGraph()
{
    clear();
}

bool StateGraph::isOK()
{
    return constructed_;
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
    }
    catch( ... )
    {
        return false;
    }
    return true;
}

void StateGraph::clear()
{
    constructed_ = false;
    nodes_.clear();
    state_view_ = StateView();
}

void StateGraph::construct()
{
    std::map<std::string, int> heights;
    for(auto& it : nodes_)
    {
        std::string group = it.second.group;
        heights[group] = std::max(heights[group], it.second.updateDepth(nodes_));
    }

    int current_offset = 0;
    for(const auto& it : heights)
    {
        state_view_.grouped_states[it.first].offset = current_offset;
        state_view_.grouped_states[it.first].states.resize(it.second + 1);
        current_offset += it.second + 1;
    }

    constructed_ = true;
}

StateView StateGraph::getStateView()
{
    return state_view_;
}

void StateGraph::updateStateView(const std::string& states)
{
    std::istringstream sin(states);
    std::string state;
    std::vector<std::string> unknown_states;

    for(auto& group : state_view_.grouped_states)
    {
        for(auto& frame : group.second.states)
        {
            frame.empty = true;
        }
    }

    while(sin >> state)
    {
        if(constructed_ && nodes_.count(state))
        {
            StateFrame& frame = state_view_.grouped_states[nodes_[state].group].states[nodes_[state].depth];
            if(frame.name == state)
            {
                ++frame.duration;
            }
            else
            {
                frame.duration = 0;
                frame.name = state;
            }
            frame.empty = false;
        }
        else
        {
            unknown_states.emplace_back(state);
        }
    }

    for(auto& group : state_view_.grouped_states)
    {
        for(auto& frame : group.second.states)
        {
            if(frame.empty)
            {
                frame.name.clear();
            }
        }
    }

    if(!unknown_states.empty())
    {
        state_view_.unknown_states = unknown_states[0];
        for(size_t i = 1; i < unknown_states.size(); ++i)
        {
            state_view_.unknown_states += " / " + unknown_states[i];
        }
    }
}

}}
