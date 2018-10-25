#ifndef AUTOWARE_LAUNCHER_MAIN_WINDOW_HPP_
#define AUTOWARE_LAUNCHER_MAIN_WINDOW_HPP_

#include "util/config.hpp"
#include <QMainWindow>

namespace autoware_launcher {

class AwMainWindow : public QMainWindow, public AwConfigMixin
{
    Q_OBJECT

    public:

        AwMainWindow();
        virtual ~AwMainWindow() = default;

        void saveConfig(AwConfig &config) override;
        void loadConfig(AwConfig &config) override;
};

}

#endif // INCLUDE GUARD
