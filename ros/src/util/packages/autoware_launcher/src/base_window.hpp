#ifndef AUTOWARE_LAUNCHER_BASE_WINDOW_HPP_
#define AUTOWARE_LAUNCHER_BASE_WINDOW_HPP_

#include <QWidget>

namespace autoware_launcher {

class AwBaseWindow : public QWidget
{
    Q_OBJECT

    public:

        AwBaseWindow();

    private:

        static constexpr int layout_margin = 16;
};

}

#endif // INCLUDE GUARD
