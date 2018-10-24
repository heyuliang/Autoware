#ifndef VEHICLE_MONITOR_DISPLAY_HPP_INCLUDED
#define VEHICLE_MONITOR_DISPLAY_HPP_INCLUDED

// headers in Autoware
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/VehicleStatus.h>

// headers in ROS
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

// headers in rviz
#ifndef Q_MOC_RUN
#include <rviz/display.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/int_property.h>
#endif

namespace autoware_rviz_plugins {

class VehicleMonitorWidget;
class VehicleMonitor final : public rviz::Display
{
    Q_OBJECT

    public:

        VehicleMonitor();
        virtual ~VehicleMonitor();

    protected:

        void onInitialize() override;
        void reset() override;
        void update(float wall_dt, float ros_dt) override;
        void onEnable() override;
        void onDisable() override;

    protected Q_SLOTS:

        void update_topic_cmd();
        void update_topic_status();
        void update_topic_mode();
        void update_speed_max();
        void update_brake_max();
        void update_accel_max();

    private:

        template<class T>
        void subscribeTopic(std::string topic, ros::Subscriber& sub, T callback);

        VehicleMonitorWidget* widget_;

        ros::Subscriber sub_cmd_;
        ros::Subscriber sub_status_;
        ros::Subscriber sub_mode_;

        rviz::RosTopicProperty* property_topic_cmd_;
        rviz::RosTopicProperty* property_topic_status_;
        rviz::RosTopicProperty* property_topic_mode_;
        rviz::IntProperty* property_speed_max_;
        rviz::IntProperty* property_brake_max_;
        rviz::IntProperty* property_accel_max_;

        void processCmd(const autoware_msgs::VehicleCmd::ConstPtr& msg);
        void processStatus(const autoware_msgs::VehicleStatus::ConstPtr& msg);
        void processMode(const std_msgs::String::ConstPtr& msg);
};

}

#endif // VEHICLE_MONITOR_DISPLAY_HPP_INCLUDED
