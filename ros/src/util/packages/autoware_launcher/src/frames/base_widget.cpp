#include "frames/base_widget.hpp"

#include <QLabel>
#include <QPainter>

namespace autoware_launcher {

AwBaseWidget::AwBaseWidget()
{
    title_ = new QLabel;
    title_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    hlayout_ = new QHBoxLayout;
    hlayout_->setSpacing(layout_margin);
    hlayout_->addWidget(title_);

    vlayout_ = new QVBoxLayout;
    vlayout_->setMargin(layout_margin);
    vlayout_->setSpacing(layout_margin * 2);
    vlayout_->addLayout(hlayout_);
    setLayout(vlayout_);
}

void AwBaseWidget::addButton(QWidget* button)
{
    hlayout_->addWidget(button);
}

void AwBaseWidget::addWidget(QWidget* widget)
{
    vlayout_->addWidget(widget);
}

void AwBaseWidget::setFrameTitle(const QString &title)
{
    title_->setText(title);
}

void AwBaseWidget::paintEvent(QPaintEvent* event)
{
    QWidget::paintEvent(event);
    QPainter painter(this);

    int bottom = layout()->itemAt(0)->geometry().bottom() + layout_margin;
    painter.drawRect(0, 0, width()-1, height()-1);
    painter.drawLine(0, bottom, width()-1, bottom);
}

}
