#include "display.hpp"
#include "widget.hpp"

namespace autoware_rviz_plugins {

VehicleMonitor::VehicleMonitor() : rviz::Display()
{
    property_topic_cmd_    = new rviz::RosTopicProperty("VehicleCmd Topic",    "", ros::message_traits::datatype<autoware_msgs::VehicleCmd>(),    "autoware_msgs::VehicleCmd topic to subscribe to.",    this, SLOT(update_topic_cmd())    );
    property_topic_status_ = new rviz::RosTopicProperty("VehicleStatus Topic", "", ros::message_traits::datatype<autoware_msgs::VehicleStatus>(), "autoware_msgs::VehicleStatus topic to subscribe to.", this, SLOT(update_topic_status()) );
    property_topic_mode_   = new rviz::RosTopicProperty("CtrlMode Topic",      "", ros::message_traits::datatype<std_msgs::String>(),             "std_msgs::String topic to subscribe to.",             this, SLOT(update_topic_mode())   );

    property_speed_max_  = new rviz::IntProperty("Max speed value",   60, "Maximum speed value.",this, SLOT(update_speed_max()) );
    property_brake_max_  = new rviz::IntProperty("Max brake value", 4095, "Maximum brake value.",this, SLOT(update_brake_max()) );
    property_accel_max_  = new rviz::IntProperty("Max accel value", 4095, "Maximum accel value.",this, SLOT(update_accel_max()) );
}

VehicleMonitor::~VehicleMonitor()
{
    if( initialized() )
    {
        delete widget_;
    }
}

void VehicleMonitor::onInitialize()
{
    widget_ = new VehicleMonitorWidget();
    setAssociatedWidget( widget_ );

    update_topic_cmd();
    update_topic_status();
    update_topic_mode();
    update_speed_max();
    update_brake_max();
    update_accel_max();

    VehicleMonitorWidget::GearList gear_list;
    XmlRpc::XmlRpcValue params;
    ros::param::get("/vehicle_info/gear/", params);
    for(auto& param : params)
    {
        gear_list.push_back( {param.first, param.second} );
    }
    widget_->configureTransmission(gear_list);
}

void VehicleMonitor::reset()
{
    return;
}

void VehicleMonitor::update(float wall_dt, float ros_dt)
{
    widget_->update();
}

void VehicleMonitor::onEnable()
{
    subscribeTopic(property_topic_cmd_->getTopicStd(), sub_cmd_, &VehicleMonitor::processCmd);
    subscribeTopic(property_topic_status_->getTopicStd(), sub_status_, &VehicleMonitor::processStatus);
    subscribeTopic(property_topic_mode_->getTopicStd(), sub_status_, &VehicleMonitor::processMode);
}

void VehicleMonitor::onDisable()
{
    sub_cmd_.shutdown();
    sub_status_.shutdown();
    sub_mode_.shutdown();
}

void VehicleMonitor::processCmd(const autoware_msgs::VehicleCmd::ConstPtr& msg)
{
    widget_->setSpeedCmd( msg->ctrl_cmd.linear_velocity );
    widget_->setAngleCmd( msg->ctrl_cmd.steering_angle * 180 / M_PI );
}

void VehicleMonitor::processStatus(const autoware_msgs::VehicleStatus::ConstPtr& msg)
{
    widget_->setSpeedStatus( msg->speed      );
    widget_->setAngleStatus( msg->angle      );
    widget_->setShiftStatus( msg->gearshift  );
    widget_->setBrakeStatus( msg->brakepedal );
    widget_->setAccelStatus( msg->drivepedal );
}

void VehicleMonitor::processMode(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == "remote")
    {
        widget_->setCtrlMode( VehicleMonitorWidget::CtrlMode::REMOTE );
    }
    else if(msg->data == "auto")
    {
        widget_->setCtrlMode( VehicleMonitorWidget::CtrlMode::AUTO );
    }
    else
    {
        widget_->setCtrlMode( VehicleMonitorWidget::CtrlMode::UNKNOWN );
    }
}

template<class T>
void VehicleMonitor::subscribeTopic(std::string topic, ros::Subscriber& sub, T callback)
{
    sub.shutdown();
    if((topic.length() > 0) && (topic != "/"))
    {
        sub = ros::NodeHandle().subscribe(topic, 1, callback, this);
    }
}

void VehicleMonitor::update_topic_cmd()
{
    subscribeTopic(property_topic_cmd_->getTopicStd(), sub_cmd_, &VehicleMonitor::processCmd);
}

void VehicleMonitor::update_topic_status()
{
    subscribeTopic(property_topic_status_->getTopicStd(), sub_status_, &VehicleMonitor::processStatus);
}

void VehicleMonitor::update_topic_mode()
{
    subscribeTopic(property_topic_mode_->getTopicStd(), sub_status_, &VehicleMonitor::processMode);
}

void VehicleMonitor::update_speed_max()
{
    widget_->configureSpeedLimit(property_speed_max_->getInt());
}

void VehicleMonitor::update_brake_max()
{
    widget_->configureBrakeLimit(property_brake_max_->getInt());
}

void VehicleMonitor::update_accel_max()
{
    widget_->configureAccelLimit(property_accel_max_->getInt());
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::VehicleMonitor, rviz::Display)
