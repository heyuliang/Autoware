#include "profile_frame.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>

namespace autoware_launcher {

AwProfileFrame::AwProfileFrame()
{
    setTitle("Description");

    auto hlayout = new QHBoxLayout;
    hlayout->setMargin(0);
    hlayout->setSpacing(4);

    auto label = new QLabel("Name:");
    auto input = new QLineEdit;
    hlayout->addWidget(label);
    hlayout->addWidget(input);
    addLayout(hlayout);
}

}
