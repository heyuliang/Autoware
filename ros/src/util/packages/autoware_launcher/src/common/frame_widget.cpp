#include "frame_widget.hpp"
#include "frame_design.hpp"

#include <QPainter>
#include <QVBoxLayout>

namespace autoware_launcher {

AwFrameWidget::AwFrameWidget()
{
    auto vlayout = new QVBoxLayout;
    vlayout->setMargin (frame_padding    );
    vlayout->setSpacing(frame_padding * 2);
    setLayout(vlayout);
}

void AwFrameWidget::addInnerWidget(QWidget* inner_widget)
{
    static_cast<QVBoxLayout*>(layout())->addWidget(inner_widget);
}

void AwFrameWidget::addInnerLayout(QLayout* inner_layout)
{
    static_cast<QVBoxLayout*>(layout())->addLayout(inner_layout);
}

void AwFrameWidget::paintEvent(QPaintEvent* event)
{
    QWidget::paintEvent(event);
    QPainter painter(this);
    for(int i = 1; i < layout()->count(); ++i)
    {
        int y1 = layout()->itemAt(i - 1)->geometry().bottom();
        int y2 = layout()->itemAt(i    )->geometry().top();
        int ym = (y1 + y2) / 2;
        painter.drawLine(0, ym, width() - 1, ym);
    }
    painter.drawRect(0, 0, width() - 1, height() - 1);
}

}
