#ifndef STATE_MONITOR_PANEL_HPP_
#define STATE_MONITOR_PANEL_HPP_

#include "graph.hpp"

#include <rviz/panel.h>
#include <std_msgs/String.h>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <string>
#include <vector>

namespace autoware_rviz_plugins {

class StateMonitor : public rviz::Panel
{
    Q_OBJECT

    public:

        StateMonitor(QWidget* parent = nullptr);

        void onInitialize() override;

    protected:

        void createStateGraph();
        void paintEvent(QPaintEvent* event) override;
        void drawWithGraph();
        void drawWithoutGraph();

    private:

        void receiveState(const std_msgs::String::ConstPtr &msg);

        ros::NodeHandle node_;
        ros::Subscriber sub_state_;
        int sub_count_;

        state_monitor::StateGraph graph_;
        std::vector<std::string> states_;
};

}

#endif // INCLUDE GUARD
