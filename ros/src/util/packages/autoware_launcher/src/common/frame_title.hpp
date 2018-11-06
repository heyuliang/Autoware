#ifndef AUTOWARE_LAUNCHER_FRAME_TITLE_HPP_
#define AUTOWARE_LAUNCHER_FRAME_TITLE_HPP_

#include <QHBoxLayout>
#include <QLabel>

namespace autoware_launcher {

class AwFrameTitle : public QHBoxLayout
{
    Q_OBJECT

    public:

        AwFrameTitle();

        void addTitleButton(QWidget* widget);
};

}

#endif // INCLUDE GUARD
