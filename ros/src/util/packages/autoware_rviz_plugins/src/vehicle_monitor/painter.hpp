#ifndef VEHICLE_MONITOR_PAINTER_HPP_INCLUDED
#define VEHICLE_MONITOR_PAINTER_HPP_INCLUDED

#include <QPainter>

namespace autoware_rviz_plugins {

class VehicleMonitorPainter final : public QPainter
{
    public:

        VehicleMonitorPainter(QPaintDevice* device);

        void drawPieChart(int x, int y, int r, int ang, int anglen, const QColor &color);
};

}

#endif // VEHICLE_MONITOR_PAINTER_HPP_INCLUDED
