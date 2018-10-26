#include "browse_launcher.hpp"

namespace autoware_launcher {

AwBrowseLauncher::AwBrowseLauncher(QWidget* parent) : QHBoxLayout(parent)
{
    apply_  = new AwLaunchButton("Apply");
    path_   = new QLineEdit();
    browse_ = new AwBrowseButton("Browse");

    addWidget(apply_);
    addWidget(path_);
    addWidget(browse_);

    apply_->setProgram("roslaunch");
    connect(browse_, &AwBrowseButton::browsed, path_, &QLineEdit::setText);
    connect(path_, &QLineEdit::textChanged, apply_, &AwLaunchButton::setNativeArguments);
}

QString AwBrowseLauncher::getPath() const
{
    return path_->text();
}

void AwBrowseLauncher::setPath(const QString& path)
{
    path_->setText(path);
}

}
