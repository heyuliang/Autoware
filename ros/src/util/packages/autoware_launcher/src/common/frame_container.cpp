#include "frame_container.hpp"
#include "frame_design.hpp"

namespace autoware_launcher {

AwFrameContainer::AwFrameContainer()
{
    auto vlayout = new QVBoxLayout;
    vlayout->setMargin (frame_margin);
    vlayout->setSpacing(frame_margin);
    setLayout(vlayout);
}

void AwFrameContainer::addFrame(QWidget* frame, int stretch)
{
    static_cast<QVBoxLayout*>(layout())->addWidget(frame, stretch);
}

void AwFrameContainer::onConfigClose()
{

}

void AwFrameContainer::onConfigApply()
{
    close();
}

void AwFrameContainer::onConfigCancel()
{
    close();
}

}
