#include "launch_button.hpp"

#include <QProcess>

namespace autoware_launcher {

AwLaunchButton::AwLaunchButton(const QString& text, QWidget* parent) : QPushButton(text, parent)
{
    process = new QProcess(this);

    setCheckable(true);
    connect(this, &AwLaunchButton::toggled, this, &AwLaunchButton::onToggled);
    connect(process, static_cast<void(QProcess::*)(int)>(&QProcess::finished), this, &AwLaunchButton::onFinished);
}

void AwLaunchButton::onToggled(bool checked)
{
    if(checked)
    {
        process->start("rosrun rviz rviz");
    }
    else
    {
        process->terminate();
    }
}

void AwLaunchButton::onFinished(int code)
{
    printf("Exit Code: %d\n", code);
    setChecked(false);
}

}
