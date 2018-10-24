#include "painter.hpp"

namespace autoware_rviz_plugins {

VehicleMonitorPainter::VehicleMonitorPainter(QPaintDevice* device) : QPainter(device)
{

}

void VehicleMonitorPainter::drawPieChart(int x, int y, int r, int ang, int anglen, const QColor &color)
{
    setBrush( color );
    drawPie(x-r, y-r, x+(2*r), y+(2*r), 16 * ang, 16 * anglen);
}

}
