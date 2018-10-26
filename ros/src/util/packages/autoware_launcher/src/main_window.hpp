#ifndef AUTOWARE_LAUNCHER_MAIN_WINDOW_HPP_
#define AUTOWARE_LAUNCHER_MAIN_WINDOW_HPP_

#include "main_config.hpp"
#include <QMainWindow>

namespace autoware_launcher {

class AwMainWindow : public QMainWindow
{
    Q_OBJECT

    public:

        AwMainWindow();
        virtual ~AwMainWindow() = default;

        void saveSettings();
        void loadSettings();

    public Q_SLOTS:

        void onLoad();
        void onSave();
        void onSaveAs();

    protected:

        void closeEvent(QCloseEvent *event) override;

    private:

        AwConfigFile config_file_;
        AwConfigInterface* config_tool_;
};

}

#endif // INCLUDE GUARD
