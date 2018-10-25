#include "browse_launcher.hpp"

#include "launch_button.hpp"
#include "browse_button.hpp"

#include <QPushButton>
#include <QLineEdit>

namespace autoware_launcher {

AwBrowseLauncher::AwBrowseLauncher(QWidget* parent) : QHBoxLayout(parent)
{
    auto apply  = new AwLaunchButton("Apply");
    auto path   = new QLineEdit();
    auto browse = new AwBrowseButton("Browse");

    addWidget(apply);
    addWidget(path);
    addWidget(browse);

    apply->setProgram("roslaunch");
    connect(browse, &AwBrowseButton::browsed, path, &QLineEdit::setText);
    connect(path, &QLineEdit::textChanged, apply, &AwLaunchButton::setNativeArguments);
}

}
