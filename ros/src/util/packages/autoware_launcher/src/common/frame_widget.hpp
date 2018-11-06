#ifndef AUTOWARE_LAUNCHER_FRAME_WIDGET_HPP_
#define AUTOWARE_LAUNCHER_FRAME_WIDGET_HPP_

#include <QWidget>

namespace autoware_launcher {

class AwFrameWidget : public QWidget
{
    Q_OBJECT

    public:

        AwFrameWidget();

        void addInnerWidget(QWidget* inner_widget);
        void addInnerLayout(QLayout* inner_layout);

    protected:

        void paintEvent(QPaintEvent* event) override;
};

}

#endif // INCLUDE GUARD
