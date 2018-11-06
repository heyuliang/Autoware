#include "base_window.hpp"

#include "base_widget.hpp"

#include <QPushButton>

namespace autoware_launcher {

AwBaseWindow::AwBaseWindow()
{
    vlayout_ = new QVBoxLayout;
    vlayout_->setMargin(layout_margin);
    vlayout_->setSpacing(layout_margin);
    vlayout_->addStretch();
    setLayout(vlayout_);

    auto apply  = new QPushButton("Apply");
    auto cancel = new QPushButton("Cancel");
    hlayout_ = new QHBoxLayout;
    hlayout_->addStretch();
    hlayout_->addWidget(cancel);
    hlayout_->addWidget(apply);

    vlayout_->addLayout(hlayout_);

    connect(apply, &QPushButton::clicked, this, &AwBaseWindow::onConfigApply);
    connect(cancel, &QPushButton::clicked, this, &AwBaseWindow::onConfigCancel);
}

void AwBaseWindow::addFrame(QWidget* frame, int stretch)
{
    vlayout_->insertWidget(vlayout_->count() - 2, frame, stretch);
}

void AwBaseWindow::onConfigClose()
{

}

void AwBaseWindow::onConfigApply()
{
    close();
}

void AwBaseWindow::onConfigCancel()
{
    close();
}

}
