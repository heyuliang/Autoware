#include "main_tool.hpp"

#include <QGroupBox>
#include <QVBoxLayout>

namespace autoware_launcher {

AwMainTool::AwMainTool(QWidget* parent) : QWidget(parent)
{
    auto layout = new QVBoxLayout;
    for(auto title : {"Vehicle", "Map", "Sensors", "Rviz"})
    {
        children[title] = new AwBrowseLauncher;

        auto group = new QGroupBox(title);
        group->setLayout(children[title]);
        group->setStyleSheet("QGroupBox { border: 1px solid gray; margin-top: 0.5em; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }");
        layout->addWidget(group);


    }
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

}
