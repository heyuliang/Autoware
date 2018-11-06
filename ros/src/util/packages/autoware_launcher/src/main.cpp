#include "main_window.hpp"

#include <QApplication>

int main(int argc, char** argv)
{
    QApplication application(argc, argv);
    autoware_launcher::AwMainWindow window;
    window.loadSettings();
    //window.setStyleSheet("font-size: 16px");
    //window.setStyleSheet("border: 1px solid red");
    window.show();
    return application.exec();
}
