#ifndef AUTOWARE_LAUNCHER_BROWSE_BUTTON_HPP_
#define AUTOWARE_LAUNCHER_BROWSE_BUTTON_HPP_

#include <QPushButton>

namespace autoware_launcher {

class AwBrowseButton : public QPushButton
{
    Q_OBJECT

    public:

        AwBrowseButton(const QString& text = "Browse", QWidget* parent = nullptr);
        virtual ~AwBrowseButton() = default;

    Q_SIGNALS:

        void browsed(const QString& path);

    private Q_SLOTS:

        void onClicked(bool checked);
};

}

#endif // INCLUDE GUARD
