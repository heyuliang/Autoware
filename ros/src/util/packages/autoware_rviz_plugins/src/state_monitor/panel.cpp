#include "panel.hpp"

#include <QPainter>

namespace autoware_rviz_plugins {

StateMonitor::StateMonitor(QWidget* parent)
{
    //QTimer *timer = new QTimer(this);
    //connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    //timer->start(1000);
}

void StateMonitor::onInitialize()
{
    sub_state_ = node_.subscribe("/decision_maker/state", 1, &StateMonitor::receiveState, this);
}

void StateMonitor::receiveState(const std_msgs::String::ConstPtr &msg)
{
    static int cnt = 0;
    state_string_ = msg->data + "\n" + std::to_string(++cnt);
    update();
}

void StateMonitor::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.drawText(0, 0, width(), height(), Qt::AlignHCenter | Qt::AlignVCenter, QString::fromStdString(state_string_));
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::StateMonitor, rviz::Panel)
