#ifndef AUTOWARE_LAUNCHER_LAUNCH_BUTTON_HPP_INCLUDED
#define AUTOWARE_LAUNCHER_LAUNCH_BUTTON_HPP_INCLUDED

#include <QPushButton>
class QProcess;

namespace autoware_launcher {

class AwLaunchButton : public QPushButton
{
    Q_OBJECT

    public:

        AwLaunchButton(const QString& text, QWidget* parent = 0);
        ~AwLaunchButton() = default;

    private Q_SLOTS:

        void onToggled(bool checked);
        void onFinished(int code);

    private:

        QProcess* process;
};

}

#endif // INCLUDE GUARD
