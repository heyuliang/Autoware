#include "base_window.hpp"

namespace autoware_launcher {

AwBaseWindow::AwBaseWindow()
{



}

/*
#include "map_config_widget.hpp"

#include "base_widget.hpp"
#include <QGroupBox>
#include <QVBoxLayout>

#define MERGIN 16

namespace autoware_launcher {

AwMainTool::AwMainTool(QWidget* parent) : QWidget(parent)
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

    auto layout = new QVBoxLayout;
    layout->setMargin(MERGIN);
    layout->setSpacing(MERGIN);
    layout->addWidget(profile);
    layout->addWidget(map);
    layout->addWidget(vehicle);
    layout->addWidget(sensors);
    layout->addStretch();
    setLayout(layout);

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
*/

}
