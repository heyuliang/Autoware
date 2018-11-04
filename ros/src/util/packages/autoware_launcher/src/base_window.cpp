#include "base_window.hpp"

#include "frames/base_widget.hpp"
#include <QVBoxLayout>

namespace autoware_launcher {

AwBaseWindow::AwBaseWindow()
{
    auto vboxlayout = new QVBoxLayout;
    vboxlayout->setMargin(layout_margin);
    vboxlayout->setSpacing(layout_margin);
    vboxlayout->addStretch();
    setLayout(vboxlayout);
}

void AwBaseWindow::addSubWidget(QWidget* widget, int stretch)
{
    QVBoxLayout* vboxlayout = static_cast<QVBoxLayout*>(layout());
    vboxlayout->insertWidget(vboxlayout->count() - 1, widget, stretch);
}

}
