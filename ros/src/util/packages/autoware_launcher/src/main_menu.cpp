#include "main_menu.hpp"

namespace autoware_launcher {

AwMainMenu::AwMainMenu(QWidget* parent) : QMenuBar(parent)
{
    QMenu* menu_file = addMenu("File");
    menu_file->addAction("Open Config");
    menu_file->addAction("Save Config");
    menu_file->addAction("Save Config As");
    menu_file->addAction("Recent Configs");
}

}
