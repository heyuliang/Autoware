#ifndef AUTOWARE_LAUNCHER_FRAME_CONTAINER_HPP_
#define AUTOWARE_LAUNCHER_FRAME_CONTAINER_HPP_

#include <QWidget>

namespace autoware_launcher {

class AwFrameContainer : public QWidget
{
    Q_OBJECT

    public:

        AwFrameContainer();

    protected:

        void addFrame(QWidget* frame, int stretch = 0);

    private Q_SLOTS:

        void onConfigClose();
        void onConfigApply();
        void onConfigCancel();
};

}

#endif // INCLUDE GUARD
