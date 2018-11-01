#include "grid.hpp"

namespace autoware_rviz_plugins {
namespace state_monitor {

Grid::Grid(int area_width, int area_height, int row, int col, int unit_width, int line_width)
{
    int draw_width  = (unit_width * col) + (line_width * (col + 1));
    int draw_height = (draw_width * area_height / area_width);

    uw_  = (unit_width);
    uh_  = (draw_height - (line_width * (row + 1))) / row;
    row_ = row;
    col_ = col;
    pad_ = line_width;
}

QRect Grid::drawArea()
{
    int w = (uw_ * col_) + (pad_ * (col_ + 1));
    int h = (uh_ * row_) + (pad_ * (row_ + 1));
    return QRect(0, 0, w, h);
}

QRect Grid::cellArea(int row, int col, int row_span, int col_span)
{
    int x = (uw_ * col     ) + (pad_ * (col      + 1));
    int y = (uh_ * row     ) + (pad_ * (row      + 1));
    int w = (uw_ * col_span) + (pad_ * (col_span - 1));
    int h = (uh_ * row_span) + (pad_ * (row_span - 1));
    return QRect(x, y, w, h);
}

}}
