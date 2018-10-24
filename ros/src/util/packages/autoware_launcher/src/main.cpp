#include "main_window.hpp"

#include <QApplication>

int main(int argc, char** argv)
{
    using namespace autoware_launcher;

    QApplication application(argc, argv);
    AwMainWindow window;
    window.show();
    return application.exec();
}
