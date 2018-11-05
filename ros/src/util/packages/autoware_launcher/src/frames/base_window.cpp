#include "frames/base_window.hpp"

#include "frames/base_widget.hpp"

namespace autoware_launcher {

AwBaseWindow::AwBaseWindow()
{
    vlayout_ = new QVBoxLayout;
    vlayout_->setMargin(layout_margin);
    vlayout_->setSpacing(layout_margin);
    vlayout_->addStretch();
    setLayout(vlayout_);
}

void AwBaseWindow::addFrame(QWidget* frame, int stretch)
{
    vlayout_->insertWidget(vlayout_->count() - 1, frame, stretch);
}

}
