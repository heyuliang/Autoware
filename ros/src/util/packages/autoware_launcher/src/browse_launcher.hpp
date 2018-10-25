#ifndef AUTOWARE_LAUNCHER_BROWSE_LAUNCHER_HPP_
#define AUTOWARE_LAUNCHER_BROWSE_LAUNCHER_HPP_

#include <QHBoxLayout>

namespace autoware_launcher {

class AwBrowseLauncher : public QHBoxLayout
{
    Q_OBJECT

    public:

        AwBrowseLauncher(QWidget* parent = nullptr);
        virtual ~AwBrowseLauncher() = default;
};

}

#endif // INCLUDE GUARD
