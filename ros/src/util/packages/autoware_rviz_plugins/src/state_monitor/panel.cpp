#include "panel.hpp"
#include "grid.hpp"
#include "graph.hpp"

#include <QPainter>

namespace autoware_rviz_plugins {
namespace state_monitor {

const QColor color_white = QColor(0xFF, 0xFF, 0xFF);
const QColor color_red   = QColor(0xAA, 0x00, 0x00);
const QColor color_green = QColor(0x00, 0xAA, 0x00);
const QColor color_blue  = QColor(0x40, 0x80, 0xC0);
const QColor color_gray  = QColor(0xAA, 0xAA, 0xAA);

Panel::Panel(QWidget* parent) : rviz::Panel(parent)
{
}

void Panel::onInitialize()
{
    createStateGraph();
    sub_state_ = node_.subscribe("/decision_maker/state", 1, &Panel::receiveState, this);
    sub_count_ = 0;
}

void Panel::createStateGraph()
{
    graph_.clear();
    for(const std::string group : {"drive", "mission", "vehicle"})
    {
        std::string filename;
        const std::string param = "/decision_maker/state_" + group + "_file_name";

        if(!node_.getParam(param, filename))
        {
            return;
        }
        if(!graph_.load(group, filename))
        {
            return;
        }
    }
    graph_.construct();
}

void Panel::receiveState(const std_msgs::String::ConstPtr &msg)
{
    states_.clear();
    unknown_states_.clear();

    std::istringstream sin(msg->data);
    std::string state;
    while(sin >> state)
    {
        //states_.push_back(state);
    }

    ++sub_count_;
    update();
}

void Panel::paintEvent(QPaintEvent* event)
{
    if(graph_.isOK())
    {
        drawWithGraph();
    }
    else
    {
        drawWithoutGraph();
    }
}

void Panel::drawWithGraph()
{

}

void Panel::drawWithoutGraph()
{
    // Painter
    Grid grid(width(), height(), 3, 8, 150, 2);
    QPainter painter(this);
    painter.setWindow(grid.drawArea());

    // Header
    {
        QString message = QString("State topic was subscribed %1 times.").arg(sub_count_);
        QRect area = grid.cellArea(0, 0, 1, 8);
        painter.setBrush(color_gray);
        painter.setPen(Qt::NoPen);
        painter.drawRect(area);
        painter.setPen(color_white);
        painter.drawText(area, Qt::AlignCenter, message);
    }

    // Group
    {
        QRect area = grid.cellArea(1, 0, 1, 8);
        painter.setBrush(color_gray);
        painter.setPen(Qt::NoPen);
        painter.drawRect(area);
        painter.setPen(color_white);
        painter.drawText(area, Qt::AlignCenter, "Failed to load the state transition.");
    }

    // State
    {
        QRect area = grid.cellArea(2, 0, 1, 8);
        painter.setBrush(color_gray);
        painter.setPen(Qt::NoPen);
        painter.drawRect(area);
        painter.setPen(color_white);
        painter.drawText(area, Qt::AlignCenter, unknown_states_.join(" / "));
    }
}

}}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::state_monitor::Panel, rviz::Panel)
