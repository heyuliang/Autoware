#ifndef AUTOWARE_LAUNCHER_MAIN_WINDOW_HPP_INCLUDED
#define AUTOWARE_LAUNCHER_MAIN_WINDOW_HPP_INCLUDED

#include <QMainWindow>

namespace autoware_launcher {

class AwMainWindow : public QMainWindow
{
    Q_OBJECT

    public:

        AwMainWindow();
        ~AwMainWindow() = default;
};

}

#endif // INCLUDE GUARD
