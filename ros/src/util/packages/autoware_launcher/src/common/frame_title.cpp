#include "frame_title.hpp"
#include "frame_design.hpp"

namespace autoware_launcher {

AwFrameTitle::AwFrameTitle()
{
    setSpacing(frame_padding);

    auto label_ = new QLabel("Title");
    label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    addWidget(label_);
}

void AwFrameTitle::addTitleButton(QWidget *widget)
{
    addWidget(widget);
}

}
