#include "widget.hpp"
#include "painter.hpp"

#include <sstream>
#include <iomanip>

namespace autoware_rviz_plugins {

const QColor color_white  = QColor(0xFF, 0xFF, 0xFF);
const QColor color_black  = QColor(0x00, 0x00, 0x00);
const QColor color_red    = QColor(0xFF, 0x00, 0x00);
const QColor color_orange = QColor(0xFF, 0xCC, 0x00);
const QColor color_blue   = QColor(0x40, 0x80, 0xC0);

VehicleMonitorWidget::VehicleMonitorWidget( QWidget* parent ) : QWidget( parent )
{
    twist_gate_ctrl_mode = CtrlMode::UNKNOWN;

    cmd_speed_kph = 0.0;
    cmd_angle_deg = 0.0;
    status_speed_kph = 0.0;
    status_angle_deg = 0.0;
    status_accel = 0;
    status_brake = 0;
    status_shift = 0;
    //status_speed_mode = -1;
    //status_angle_mode = -1;

    config_speed_limit =  120;
    config_brake_limit = 4096;
    config_accel_limit = 4096;
}

void VehicleMonitorWidget::setCtrlMode( CtrlMode ctrl_mode )
{
    twist_gate_ctrl_mode = ctrl_mode;
}

void VehicleMonitorWidget::setSpeedCmd( double kph )
{
    cmd_speed_kph = kph;
}

void VehicleMonitorWidget::setAngleCmd( double deg )
{
    cmd_angle_deg = deg;
}

void VehicleMonitorWidget::setSpeedStatus( double kph )
{
    status_speed_kph = kph;
}

void VehicleMonitorWidget::setAngleStatus( double deg )
{
    status_angle_deg = deg;
}

void VehicleMonitorWidget::setShiftStatus( double value )
{
    status_shift = value;
}

void VehicleMonitorWidget::setBrakeStatus( double value )
{
    status_brake = value;
}

void VehicleMonitorWidget::setAccelStatus( double value )
{
    status_accel = value;
}

void VehicleMonitorWidget::configureSpeedLimit( int limit )
{
    config_speed_limit = limit;
}

void VehicleMonitorWidget::configureBrakeLimit( int limit )
{
    config_brake_limit = limit;
}

void VehicleMonitorWidget::configureAccelLimit( int limit )
{
    config_accel_limit = limit;
}

void VehicleMonitorWidget::configureTransmission( const GearList& gear_list )
{
    config_gear_list = gear_list;
}

void VehicleMonitorWidget::paintEvent(QPaintEvent *event)
{
    VehicleMonitorPainter painter(this);
    const char* unit_kph = " km/h";
    const char* unit_mps = " m/s";
    const char* unit_deg = " deg";
    const char* unit_rad = " rad";

    int text_height = width() / 8;
    int curr_height = 0;
    painter.setBrush(QColor(0x00, 0x00, 0x00));
    painter.drawRect(0, 0, width(), height());

    // Status
    {
        painter.setViewport(0, curr_height, width(), text_height);
        curr_height += text_height;
        drawControlMode( painter );
    }

    // Speed
    {
        painter.setViewport(0, curr_height, width(), width()*2/3);
        curr_height += width()*2/3;
        drawSpeedGraph( painter );

        QString cmd_kph = toString(cmd_speed_kph,       0) + unit_kph;
        QString cmd_mps = toString(cmd_speed_kph / 3.6, 1) + unit_mps;
        painter.setViewport(0, curr_height, width(), text_height);
        curr_height += text_height;
        drawDualUnitsText( painter, QColor(0xFF, 0xFF, 0xFF), "cmd", cmd_kph, cmd_mps );

        QString status_kph = toString(status_speed_kph,       0) + unit_kph;
        QString status_mps = toString(status_speed_kph / 3.6, 1) + unit_mps;
        painter.setViewport(0, curr_height, width(), text_height);
        curr_height += text_height;
        drawDualUnitsText( painter, QColor(0xFF, 0xCC, 0x00), "status", status_kph, status_mps );
    }

    // Angle
    {
        painter.setViewport(0, curr_height, width(), width()*5/6);
        curr_height += width()*5/6;
        drawAngleGraph( painter );

        QString cmd_deg = toString(cmd_angle_deg,              0) + unit_deg;
        QString cmd_rad = toString(cmd_angle_deg * M_PI / 180, 1) + unit_rad;
        painter.setViewport(0, curr_height, width(), text_height);
        curr_height += text_height;
        drawDualUnitsText( painter, QColor(0xFF, 0xFF, 0xFF), "cmd", cmd_deg, cmd_rad );

        QString status_deg = toString(status_angle_deg,              0) + unit_deg;
        QString status_rad = toString(status_angle_deg * M_PI / 180, 2) + unit_rad;
        painter.setViewport(0, curr_height, width(), text_height);
        curr_height += text_height;
        drawDualUnitsText( painter, QColor(0xFF, 0xCC, 0x00), "status", status_deg, status_rad );
    }

    // Pedal
    painter.setViewport(0, curr_height, width(),  width()/2);
    curr_height += width()/2;
    drawPedal( painter );

    // Shift
    painter.setViewport(0, curr_height, width(), text_height);
    curr_height += text_height;
    drawShift( painter );

    // Others
    int rest_height = height() - curr_height;
    painter.setViewport(0, curr_height, width(), rest_height);
    painter.setWindow(0, 0, 200, 200 * rest_height / width());
}


void VehicleMonitorWidget::drawControlMode( VehicleMonitorPainter& painter )
{
    constexpr QRect rect_remote = QRect(  2, 0, 97, 23);
    constexpr QRect rect_auto   = QRect(101, 0, 97, 23);

    painter.setWindow(0, -2, 200, 27);

    painter.setPen(Qt::NoPen);
    painter.setBrush( (twist_gate_ctrl_mode == CtrlMode::REMOTE) ? QColor(0xFF, 0xCC, 0x00) : QColor(0x40, 0x80, 0xC0) );
    painter.drawRect( rect_remote );
    painter.setBrush( (twist_gate_ctrl_mode == CtrlMode::AUTO  ) ? QColor(0xFF, 0xCC, 0x00) : QColor(0x40, 0x80, 0xC0) );
    painter.drawRect( rect_auto   );

    painter.setPen(QColor(0xFF, 0xFF, 0xFF));
    painter.setBrush(Qt::NoBrush);
    painter.drawText( rect_remote, Qt::AlignHCenter | Qt::AlignVCenter, "REMOTE");
    painter.drawText( rect_auto,   Qt::AlignHCenter | Qt::AlignVCenter, "AUTO"  );
}

void VehicleMonitorWidget::drawSpeedGraph( VehicleMonitorPainter& painter )
{
    painter.setWindow(-120, -130, 240, 160);

    // Pie chart of command
    {
        int ang = 180 * cmd_speed_kph / config_speed_limit;
        painter.setPen(Qt::NoPen);
        painter.drawPieChart(0, 0, 85, 180, -180, color_blue  );
        painter.drawPieChart(0, 0, 85, 180, -ang, color_white );
        painter.drawPieChart(0, 0, 75, 180, -180, color_black );
    }

    // Pie chart of status
    {
        int ang = 180 * status_speed_kph / config_speed_limit;
        painter.setPen(Qt::NoPen);
        painter.drawPieChart(0, 0, 70, 180, -180, color_blue  );
        painter.drawPieChart(0, 0, 70, 180, -ang, color_orange);
        painter.drawPieChart(0, 0, 50, 180, -180, color_black );
    }

    // Speed text
    {
        painter.setPen(QColor(0xFF, 0xFF, 0xFF));
        painter.setBrush(Qt::NoBrush);
        painter.drawText( -100 - 20,    0 - 20, 40, 40 , Qt::AlignHCenter | Qt::AlignVCenter, QString::number(config_speed_limit * 0 / 4));
        painter.drawText(  -70 - 20,  -70 - 20, 40, 40 , Qt::AlignHCenter | Qt::AlignVCenter, QString::number(config_speed_limit * 1 / 4));
        painter.drawText(    0 - 20, -100 - 20, 40, 40 , Qt::AlignHCenter | Qt::AlignVCenter, QString::number(config_speed_limit * 2 / 4));
        painter.drawText(   70 - 20,  -70 - 20, 40, 40 , Qt::AlignHCenter | Qt::AlignVCenter, QString::number(config_speed_limit * 3 / 4));
        painter.drawText(  100 - 20,    0 - 20, 40, 40 , Qt::AlignHCenter | Qt::AlignVCenter, QString::number(config_speed_limit * 4 / 4));
    }
}

void VehicleMonitorWidget::drawDualUnitsText( VehicleMonitorPainter& painter, const QColor& color, const QString& title, const QString& unit1, const QString& unit2 )
{
    constexpr QRect rect_title = QRect(  2, 0, 48, 23);
    constexpr QRect rect_unit1 = QRect( 52, 0, 72, 23);
    constexpr QRect rect_unit2 = QRect(126, 0, 72, 23);

    painter.setWindow(0, 0, 200, 25);

    painter.setPen( Qt::NoPen );
    painter.setBrush( QColor(0x40, 0x80, 0xC0) );
    painter.drawRect( rect_title );
    painter.drawRect( rect_unit1 );
    painter.drawRect( rect_unit2 );

    painter.setPen( color );
    painter.setBrush( Qt::NoBrush );
    painter.drawText( rect_title, Qt::AlignHCenter | Qt::AlignVCenter, title );
    painter.drawText( rect_unit1, Qt::AlignHCenter | Qt::AlignVCenter, unit1 );
    painter.drawText( rect_unit2, Qt::AlignHCenter | Qt::AlignVCenter, unit2 );
}

void VehicleMonitorWidget::drawAngleGraph( VehicleMonitorPainter& painter )
{
    const QColor colors_cmd[] = {color_blue, color_white,  color_red};
    const QColor colors_sts[] = {color_blue, color_orange, color_red};

    painter.setWindow(-120, -100, 240, 200);

    // Pie chart of command
    {
        int ang = cmd_angle_deg;
        int idx = 0;
        if(ang > +360) { ang -= 360; ++idx; }
        if(ang < -360) { ang += 360; ++idx; }
        painter.setPen(Qt::NoPen);
        painter.drawPieChart(0, 0, 85, 90, 360, colors_cmd[idx] );
        painter.drawPieChart(0, 0, 85, 90, ang, colors_cmd[idx + 1] );
        painter.drawPieChart(0, 0, 75, 90, 360, color_black );
    }

    // Pie chart of status
    {
        int ang = status_angle_deg;
        int idx = 0;
        if(ang > +360) { ang -= 360; ++idx; }
        if(ang < -360) { ang += 360; ++idx; }
        painter.setPen(Qt::NoPen);
        painter.drawPieChart(0, 0, 70, 90, 360, colors_sts[idx] );
        painter.drawPieChart(0, 0, 70, 90, ang, colors_sts[idx + 1] );
        painter.drawPieChart(0, 0, 50, 90, 360, color_black );
    }
}
void VehicleMonitorWidget::drawPedal( VehicleMonitorPainter& painter )
{
    painter.setWindow(0, 0, 240, 120);

    painter.setPen(Qt::NoPen);
    painter.setBrush( QColor(0x40, 0x80, 0xC0) );
    painter.drawRect(  40, 90, 60, -70 );
    painter.drawRect( 140, 90, 60, -70 );
    painter.setBrush( QColor(0xFF, 0xCC, 0x00) );
    painter.drawRect(  40, 90, 60, -70 * status_brake / config_brake_limit );
    painter.drawRect( 140, 90, 60, -70 * status_accel / config_accel_limit );

    painter.setPen(QColor(0xFF, 0xFF, 0xFF));
    painter.setBrush(Qt::NoBrush);
    painter.drawText(  40, 90, 60, -70, Qt::AlignHCenter | Qt::AlignVCenter, QString::number(status_brake) );
    painter.drawText( 140, 90, 60, -70, Qt::AlignHCenter | Qt::AlignVCenter, QString::number(status_accel) );
    painter.drawText(  40, 90, 60,  20, Qt::AlignHCenter | Qt::AlignVCenter, "BRAKE");
    painter.drawText( 140, 90, 60,  20, Qt::AlignHCenter | Qt::AlignVCenter, "ACCEL");
}

void VehicleMonitorWidget::drawShift( VehicleMonitorPainter& painter )
{
    if( config_gear_list.empty() )
    {
        painter.setWindow(0, 0, 200, 25);

        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(0x40, 0x80, 0xC0));
        painter.drawRect(2, 0, 196, 23);

        painter.setPen(QColor(0xFF, 0xFF, 0xFF));
        painter.setBrush(Qt::NoBrush);
        painter.drawText( 2, 0, 196, 23, Qt::AlignHCenter | Qt::AlignVCenter, "No gear information");
    }
    else
    {
        int gear_count = config_gear_list.size();
        int gear_width = 200 / gear_count;
        int area_width = (gear_width * gear_count) + 2;

        painter.setWindow(0, 0, area_width, 25);

        painter.setPen(Qt::NoPen);
        for(int i = 0; i < gear_count; ++i)
        {
            if(status_shift == config_gear_list[i].value)
            {
                painter.setBrush(QColor(0xFF, 0xCC, 0x00));
            }
            else
            {
                painter.setBrush(QColor(0x40, 0x80, 0xC0));
            }
            painter.drawRect( (i * gear_width) + 2, 0, gear_width - 2, 25 );
        }

        painter.setPen(QColor(0xFF, 0xFF, 0xFF));
        painter.setBrush(Qt::NoBrush);
        for(int i = 0; i < gear_count; ++i)
        {
            painter.drawText( (i * gear_width) + 2, 0, gear_width - 2, 25, Qt::AlignHCenter | Qt::AlignVCenter, QString::fromStdString(config_gear_list[i].name) );
        }
    }
}

QString VehicleMonitorWidget::toString(double value, int precision)
{
    std::ostringstream sout;
    sout << std::fixed << std::setprecision(precision) << value;
    return QString::fromStdString( sout.str() );
}

}
