#include "map_config_window.hpp"

#include "frames/base_widget.hpp"

namespace autoware_launcher {

AwMapConfigWindow::AwMapConfigWindow()
{
    setWindowTitle("Map Config");

    addSubWidget(new AwBaseWidget);
}

}
