#ifndef STATE_MONITOR_PANEL_HPP_
#define STATE_MONITOR_PANEL_HPP_

#include <rviz/panel.h>
#include <std_msgs/String.h>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

namespace autoware_rviz_plugins {

class StateMonitor : public rviz::Panel
{
    Q_OBJECT

    public:

        StateMonitor(QWidget* parent = nullptr);
        void onInitialize() override;

    protected:

        void paintEvent(QPaintEvent* event) override;

    private:

        void receiveState(const std_msgs::String::ConstPtr &msg);

        ros::NodeHandle node_;
        ros::Subscriber sub_state_;

        std::string state_string_;
};

}

#endif // INCLUDE GUARD
