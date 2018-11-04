#include "main_tool.hpp"

#include "map_config_widget.hpp"

#include "frames/base_widget.hpp"
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

    auto sensors = new AwBaseWidget;
    sensors->addButton( new QPushButton("Change") );
    sensors->addButton( new QPushButton("Launch") );

    addSubWidget(profile);
    addSubWidget(map);
    addSubWidget(vehicle);
    addSubWidget(sensors);
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
