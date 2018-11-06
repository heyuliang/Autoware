#ifndef AUTOWARE_LAUNCHER_BASE_WINDOW_HPP_
#define AUTOWARE_LAUNCHER_BASE_WINDOW_HPP_

#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace autoware_launcher {

class AwBaseWindow : public QWidget
{
    Q_OBJECT

    public:

        AwBaseWindow();

    protected:

        void addFrame(QWidget* frame, int stretch = 0);

    private Q_SLOTS:

        void onConfigClose();
        void onConfigApply();
        void onConfigCancel();

    private:

        static constexpr int layout_margin = 16;

        QHBoxLayout* hlayout_;
        QVBoxLayout* vlayout_;
};

}

#endif // INCLUDE GUARD
