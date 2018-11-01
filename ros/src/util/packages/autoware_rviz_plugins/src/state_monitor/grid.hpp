#ifndef STATE_MONITOR_GRID_HPP_
#define STATE_MONITOR_GRID_HPP_

#include <QRect>

namespace autoware_rviz_plugins {
namespace state_monitor {

class Grid
{
    public:

        Grid(int area_width, int area_height, int row, int col, int unit_width, int line_width);

        QRect drawArea();
        QRect cellArea(int row, int col, int row_span, int col_span);

    private:

        int uw_;
        int uh_;
        int row_;
        int col_;
        int pad_;
};

}}

#endif // INCLUDE GUARD
