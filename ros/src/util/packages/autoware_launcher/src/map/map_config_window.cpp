#include "map_config_window.hpp"

#include "profile_frame.hpp"
#include "single_file_selector.hpp"
#include "multi_file_selector.hpp"

#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>

namespace autoware_launcher {

AwMapConfigWindow::AwMapConfigWindow()
{
    setWindowTitle("Map Config");

    addFrame(new AwProfileFrame);
    addFrame(new AwSingleFileSelector("Transform"));
    addFrame(new AwMultiFileSelector("Point Cloud"));
    addFrame(new AwMultiFileSelector("Vector Map"));
}

}
