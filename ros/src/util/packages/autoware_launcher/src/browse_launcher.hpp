#ifndef AUTOWARE_LAUNCHER_BROWSE_LAUNCHER_HPP_
#define AUTOWARE_LAUNCHER_BROWSE_LAUNCHER_HPP_

#include "launch_button.hpp"
#include "browse_button.hpp"

#include <QPushButton>
#include <QLineEdit>
#include <QHBoxLayout>

namespace autoware_launcher {

class AwBrowseLauncher : public QHBoxLayout
{
    Q_OBJECT

    public:

        AwBrowseLauncher(QWidget* parent = nullptr);
        virtual ~AwBrowseLauncher() = default;

        QString getPath() const;
        void setPath(const QString& path);

    private:

        AwLaunchButton* apply_;
        QLineEdit* path_;
        AwBrowseButton* browse_;

};

}

#endif // INCLUDE GUARD
