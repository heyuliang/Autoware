#include "main_window.hpp"

#include <QApplication>

int main(int argc, char** argv)
{
    QApplication application(argc, argv);
    autoware_launcher::AwMainWindow window;
    window.loadSettings();
    window.show();
    return application.exec();
}
