#include "map_config_widget.hpp"
#include "map_config_window.hpp"

#include <QPushButton>

namespace autoware_launcher {

AwMapConfigWidget::AwMapConfigWidget()
{
    auto config_button = new QPushButton("Config");
    auto launch_button = new QPushButton("Launch");

    addButton(config_button);
    addButton(launch_button);

    connect(config_button, &QPushButton::clicked, this, &AwMapConfigWidget::test);

}

void AwMapConfigWidget::test()
{
    auto config_window = new AwMapConfigWindow;
    config_window->adjustSize();
    config_window->move(window()->pos() + window()->rect().center() - config_window->rect().center());
    config_window->setAttribute(Qt::WA_DeleteOnClose);
    config_window->setWindowModality(Qt::ApplicationModal);
    config_window->show();
}

}
