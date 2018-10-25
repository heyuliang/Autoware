#ifndef AUTOWARE_LAUNCHER_LAUNCH_BUTTON_HPP_
#define AUTOWARE_LAUNCHER_LAUNCH_BUTTON_HPP_

#include <QPushButton>
class QProcess;

namespace autoware_launcher {

class AwLaunchButton : public QPushButton
{
    Q_OBJECT

    public:

        AwLaunchButton(const QString& text, QWidget* parent = nullptr);
        virtual ~AwLaunchButton() = default;

    public Q_SLOTS:

        void setProgram(const QString& program);
        void setNativeArguments(const QString& arguments);

    private Q_SLOTS:

        void onToggled(bool checked);
        void onFinished(int code);

    private:

        QProcess* process_;
};

}

#endif // INCLUDE GUARD
