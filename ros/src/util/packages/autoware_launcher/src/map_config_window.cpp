#include "map_config_window.hpp"

#include "frames/single_file_selector.hpp"
#include "frames/multi_file_selector.hpp"

#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>

namespace autoware_launcher {

AwMapConfigWindow::AwMapConfigWindow()
{
    setWindowTitle("Map Config");

    auto hl = new QHBoxLayout;
    hl->addWidget(new QLabel("Name:"));
    hl->addWidget(new QLineEdit);
    auto w = new QWidget;
    w->setLayout(hl);
    addFrame(w);

    addFrame(new AwSingleFileSelector("Transform"));
    addFrame(new AwMultiFileSelector("Point Cloud"));
    addFrame(new AwMultiFileSelector("Vector Map"));
}

}
