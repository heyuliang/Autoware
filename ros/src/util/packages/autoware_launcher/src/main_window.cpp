#include "main_window.hpp"
#include "main_menu.hpp"

// Temporary
#include "browse_launcher.hpp"
#include <QGroupBox>

namespace autoware_launcher {

AwMainWindow::AwMainWindow()
{
    setWindowTitle("Autoware Launcher");

    setMenuBar(new AwMainMenu);

    auto layout = new QVBoxLayout;
    auto widget = new QWidget;
    for(auto title : {"Vehicle", "Map", "Sensors", "Rviz"})
    {
        auto group = new QGroupBox(title);
        group->setLayout(new AwBrowseLauncher);
        group->setStyleSheet("QGroupBox { border: 1px solid gray; margin-top: 0.5em; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }");

        layout->addWidget(group);
    }
    widget->setLayout(layout);
    setCentralWidget(widget);
}

void AwMainWindow::saveConfig(AwConfig &config)
{

}

void AwMainWindow::loadConfig(AwConfig &config)
{

}

}
