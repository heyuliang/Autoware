#include "launch_button.hpp"

#include <QProcess>

namespace autoware_launcher {

AwLaunchButton::AwLaunchButton(const QString& text, QWidget* parent) : QPushButton(text, parent)
{
    process_ = new QProcess(this);

    setCheckable(true);
    connect(this, &AwLaunchButton::toggled, this, &AwLaunchButton::onToggled);
    connect(process_, static_cast<void(QProcess::*)(int)>(&QProcess::finished), this, &AwLaunchButton::onFinished);
}

void AwLaunchButton::setProgram(const QString& program)
{
    process_->setProgram(program);
}

void AwLaunchButton::setNativeArguments(const QString& arguments)
{

    process_->setArguments(QStringList(arguments));
}

void AwLaunchButton::onToggled(bool checked)
{
    if(checked)
    {
        process_->start();
    }
    else
    {
        process_->terminate();
    }
}

void AwLaunchButton::onFinished(int code)
{
    printf("Exit Code: %d\n", code);
    setChecked(false);
}

}
