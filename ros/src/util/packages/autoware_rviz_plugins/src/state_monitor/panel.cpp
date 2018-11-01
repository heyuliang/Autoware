#include "panel.hpp"
#include "graph.hpp"

#include <QPainter>

#include <sstream>

namespace autoware_rviz_plugins {

StateMonitor::StateMonitor(QWidget* parent)
{

}

void StateMonitor::onInitialize()
{
    createStateGraph();
    sub_state_ = node_.subscribe("/decision_maker/state", 1, &StateMonitor::receiveState, this);
    sub_count_ = 0;
}

void StateMonitor::createStateGraph()
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

void StateMonitor::receiveState(const std_msgs::String::ConstPtr &msg)
{
    std::string state;
    std::istringstream sin(msg->data);

    states_.clear();
    while(sin >> state)
    {
        states_.push_back(state);
    }

    ++sub_count_;
    update();
}

void StateMonitor::paintEvent(QPaintEvent* event)
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

void StateMonitor::drawWithGraph()
{
    const QColor color_white = QColor(0xFF, 0xFF, 0xFF);
    const QColor color_red   = QColor(0xAA, 0x00, 0x00);
    const QColor color_green = QColor(0x00, 0xAA, 0x00);
    const QColor color_blue  = QColor(0x40, 0x80, 0xC0);
    const QColor color_gray  = QColor(0xAA, 0xAA, 0xAA);

    std::map<std::string, int> group_offset;
    std::map<std::string, int> group_height = graph_.getHeight();
    int state_count = 0;

    for(auto& group : group_height)
    {
        group_offset[group.first] = state_count;
        ++group.second;
        state_count += group.second;
    }

    std::string unknown_states;
    std::vector<std::string> state_list(state_count);
    for(const auto& state : states_)
    {
        state_monitor::StateNode node;
        if(graph_.getState(state, node))
        {
            int offset = group_offset[node.graph_];
            state_list[offset + node.depth_] = state;
        }
        else
        {
            unknown_states += state + ", ";
        }
    }


    QColor message_color = color_green;
    std::string message =  " Topic was subscribed " + std::to_string(sub_count_) + " times.";
    if(!unknown_states.empty())
    {
        message_color = color_red;
        message += " There are unknown states: " + unknown_states;
    }

    const int spacing_size = 2;
    const int state_width = 150;
    const int area_width  = (state_width * state_count) + (spacing_size * (state_count + 1));
    const int area_height = area_width * height() / width();
    const int unit_height = (area_height - (spacing_size * 4)) / 3;
    QPainter painter(this);
    painter.setWindow(0, 0, area_width, area_height);

    int curr_height = spacing_size;
    int curr_width  = spacing_size;

    painter.setBrush(message_color);
    painter.setPen(Qt::NoPen);
    painter.drawRect(curr_width, curr_height, area_width - (2 * spacing_size), unit_height);
    painter.setPen(color_white);
    painter.drawText(curr_width, curr_height, area_width - (2 * spacing_size), unit_height, Qt::AlignCenter, QString::fromStdString(message));

    curr_height += spacing_size + unit_height;
    curr_width = spacing_size;
    for(const auto& group : group_height)
    {
        int group_width = (group.second * (spacing_size + state_width)) - spacing_size;
        painter.setBrush(color_blue);
        painter.setPen(Qt::NoPen);
        painter.drawRect(curr_width, curr_height, group_width, unit_height);
        painter.setPen(color_white);
        painter.drawText(curr_width, curr_height, group_width, unit_height, Qt::AlignCenter, QString::fromStdString(group.first + " state"));

        curr_width += spacing_size + group_width;
    }

    curr_height += spacing_size + unit_height;
    curr_width = spacing_size;
    for(const auto& state : state_list)
    {
        painter.setBrush(state.empty() ? color_gray : color_blue);
        painter.setPen(Qt::NoPen);
        painter.drawRect(curr_width, curr_height, state_width, unit_height);
        painter.setPen(color_white);
        painter.drawText(curr_width, curr_height, state_width, unit_height, Qt::AlignCenter, QString::fromStdString(state));

        curr_width += spacing_size + state_width;
    }
}

void StateMonitor::drawWithoutGraph()
{
    int area_width  = 1200;
    int area_height = area_width * height() / width();

    std::string message =  "Failed to load the state transition. Current states are following.";
    message += " Topic was subscribed " + std::to_string(sub_count_) + " times.\n\n";
    for(const std::string& state : states_)
    {
        message += state + ", ";
    }

    QPainter painter(this);
    painter.setWindow(0, 0, area_width, area_height);
    painter.drawText(10, 10,  area_width, area_height, Qt::TextWordWrap, QString::fromStdString(message));
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::StateMonitor, rviz::Panel)
