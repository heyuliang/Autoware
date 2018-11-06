#include "main_tool.hpp"

#include "map_config_widget.hpp"

#include "base_widget.hpp"
#include "sensor_fusion_frame.hpp"
#include <QGroupBox>
#include <QVBoxLayout>

namespace autoware_launcher {

AwMainTool::AwMainTool(QWidget* parent)
{
    auto profile = new AwBaseWidget;
    profile->addButton( new QPushButton("Change") );

    auto map = new AwMapConfigWidget;

    auto vehicle = new AwBaseWidget;
    vehicle->addButton( new QPushButton("Change") );
    vehicle->addButton( new QPushButton("Launch") );

    auto sensors = new AwSensorFusionFrame;

    addFrame(profile);
    addFrame(map);
    addFrame(vehicle);
    addFrame(sensors);
}

void AwMainTool::saveConfig(AwConfig &config) const
{
    for(auto it = children.begin(); it != children.end(); ++it)
    {
        AwBrowseLauncher* launcher = it.value();
        QString key = it.key();
        config[key] = launcher->getPath();
    }
}

void AwMainTool::loadConfig(const AwConfig &config)
{
    for(auto it = children.begin(); it != children.end(); ++it)
    {
        AwBrowseLauncher* launcher = it.value();
        QString key = it.key();
        launcher->setPath(config[key].toString());
    }
}

}
