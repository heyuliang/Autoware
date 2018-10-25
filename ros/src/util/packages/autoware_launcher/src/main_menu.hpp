#ifndef AUTOWARE_LAUNCHER_MAIN_MENU_HPP_
#define AUTOWARE_LAUNCHER_MAIN_MENU_HPP_

#include <QMenuBar>

namespace autoware_launcher {

class AwMainMenu : public QMenuBar
{
    Q_OBJECT

    public:

        AwMainMenu(QWidget* parent = nullptr);
        virtual ~AwMainMenu() = default;
};

}

#endif // INCLUDE GUARD
