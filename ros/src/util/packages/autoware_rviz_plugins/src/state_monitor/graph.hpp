#ifndef STATE_MONITOR_GRAPH_HPP_
#define STATE_MONITOR_GRAPH_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>

namespace autoware_rviz_plugins {
namespace state_monitor {

struct StateFrame
{
    StateFrame();

    int duration;
    bool empty;
    std::string name;
};

struct StateGroup
{
    int offset;
    std::vector<StateFrame> states;
};

struct StateView
{
    std::string unknown_states;
    std::map<std::string, StateGroup> grouped_states;
};

struct StateNode
{
    StateNode() = default;
    StateNode(const std::string& group, const std::string& state, const std::string& parent);

    int updateDepth(std::map<std::string, StateNode>& nodes);

    int depth;
    std::string group;
    std::string state;
    std::string parent;
};

class StateGraph
{
    public:

        StateGraph();

        bool isOK();
        bool load(const std::string& graph, const std::string& filename);
        void clear();
        void construct();

        StateView getStateView();
        void updateStateView(const std::string& states);

    private:

        bool constructed_;
        std::map<std::string, StateNode> nodes_;
        StateView state_view_;
};

}}

#endif // INCLUDE GUARD
