#include "panel.hpp"
#include "grid.hpp"
#include "graph.hpp"

#include <QPainter>

namespace autoware_rviz_plugins {
namespace state_monitor {

const QColor color_white  = QColor(0xFF, 0xFF, 0xFF);
const QColor color_red    = QColor(0xAA, 0x00, 0x00);
const QColor color_green  = QColor(0x00, 0xAA, 0x00);
const QColor color_blue   = QColor(0x40, 0x80, 0xC0);
const QColor color_gray   = QColor(0xAA, 0xAA, 0xAA);
const QColor color_purple = QColor(0x80, 0x40, 0xC0);
const std::vector<std::string> group_names  = {"drive", "mission", "vehicle"};

inline QColor mixColor(const QColor& color1, int ratio1, const QColor& color2, int ratio2)
{
    int total_ratio = ratio1 + ratio2;
    int r = (color1.red  () * ratio1) + (color2.red  () * ratio2);
    int g = (color1.green() * ratio1) + (color2.green() * ratio2);
    int b = (color1.blue () * ratio1) + (color2.blue () * ratio2);
    return QColor(r / total_ratio, g / total_ratio, b / total_ratio);
}



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
    for(const auto& group_name : group_names)
    {
        std::string filename;
        const std::string param = "/decision_maker/state_" + group_name + "_file_name";

        if(!node_.getParam(param, filename))
        {
            return;
        }
        if(!graph_.load(group_name, filename))
        {
            return;
        }
    }
    graph_.construct();
}

void Panel::receiveState(const std_msgs::String::ConstPtr &msg)
{
    ++sub_count_;
    graph_.updateStateView(msg->data);
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
    StateView state_view = graph_.getStateView();
    int grid_column = 0;
    for(const auto& group : state_view.grouped_states)
    {
        grid_column += group.second.states.size();
    }

    Grid grid(width(), height(), 3, grid_column, 150, 2);
    QPainter painter(this);
    painter.setWindow(grid.drawArea());

    // Header
    {
        QString message = QString("State topic was subscribed %1 times.").arg(sub_count_);
        QColor bgcolor = color_green;
        if(!state_view.unknown_states.empty())
        {
            bgcolor = color_red;
            message += QString::fromStdString(" There are unknown states: " + state_view.unknown_states);
        }

        QRect area = grid.cellArea(0, 0, 1, 8);
        painter.setBrush(bgcolor);
        painter.setPen(Qt::NoPen);
        painter.drawRect(area);
        painter.setPen(color_white);
        painter.drawText(area, Qt::AlignCenter, message);
    }

    // Group
    bool color_flag = true;
    for(const auto& group_name : group_names)
    {
        StateGroup group = state_view.grouped_states[group_name];

        QRect area = grid.cellArea(1, group.offset, 1, group.states.size());
        painter.setBrush(color_flag ? color_blue : color_purple);
        painter.setPen(Qt::NoPen);
        painter.drawRect(area);
        painter.setPen(color_white);
        painter.drawText(area, Qt::AlignCenter, QString::fromStdString(group_name));

        // State
        for(size_t i = 0; i < group.states.size(); ++i)
        {
            QColor bgcolor = color_gray;
            if(!group.states[i].empty)
            {
                constexpr int max_ratio = 5;
                int ratio = std::min(max_ratio, group.states[i].duration);
                bgcolor = color_flag ? color_blue : color_purple;
                bgcolor = mixColor(bgcolor, ratio, color_red, max_ratio - ratio);
            }

            QRect area = grid.cellArea(2, group.offset + i, 1, 1);
            painter.setBrush(bgcolor);
            painter.setPen(Qt::NoPen);
            painter.drawRect(area);
            painter.setPen(color_white);
            painter.drawText(area, Qt::AlignCenter, QString::fromStdString(group.states[i].name));
        }

        color_flag = !color_flag;
    }
}

void Panel::drawWithoutGraph()
{
    StateView state_view = graph_.getStateView();

    Grid grid(width(), height(), 3, 8, 150, 2);
    QPainter painter(this);
    painter.setWindow(grid.drawArea());

    // Header
    {
        QString message = QString("State topic was subscribed %1 times.").arg(sub_count_);
        QRect area = grid.cellArea(0, 0, 1, 8);
        painter.setBrush(color_green);
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
        painter.drawText(area, Qt::AlignCenter, "Failed to load the state transition. There are following states");
    }

    // State
    {
        QRect area = grid.cellArea(2, 0, 1, 8);
        painter.setBrush(color_gray);
        painter.setPen(Qt::NoPen);
        painter.drawRect(area);
        painter.setPen(color_white);
        painter.drawText(area, Qt::AlignCenter, QString::fromStdString(state_view.unknown_states));
    }
}

}}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::state_monitor::Panel, rviz::Panel)
