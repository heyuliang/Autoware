#include "main_window.hpp"

// Temporary
#include "launch_button.hpp"

namespace autoware_launcher {

AwMainWindow::AwMainWindow()
{
    setWindowTitle("Autoware Launcher");
    setCentralWidget(new AwLaunchButton("rviz"));
}

}
