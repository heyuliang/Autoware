#ifndef STATE_MONITOR_GRAPH_HPP_
#define STATE_MONITOR_GRAPH_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>

namespace autoware_rviz_plugins {
namespace state_monitor {

class StateNode
{
    public:

        StateNode() = default;
        StateNode(const std::string& graph, const std::string& state, const std::string& parent);

        int updateDepth(std::map<std::string, StateNode>& nodes);

        int depth_;
        std::string graph_;
        std::string state_;
        std::string parent_;
};

class StateGraph
{
    public:

        StateGraph();

        bool isOK();
        bool load(const std::string& graph, const std::string& filename);
        void clear();
        void construct();

        bool getState(std::string state, StateNode& node);
        std::map<std::string, int> getHeight();

    private:

        bool ok_;
        std::map<std::string, int> height_;
        std::map<std::string, StateNode> nodes_;
};

}}

#endif // INCLUDE GUARD
