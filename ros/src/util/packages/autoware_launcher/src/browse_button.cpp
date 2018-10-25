#include "browse_button.hpp"

#include <QFileDialog>

namespace autoware_launcher {

AwBrowseButton::AwBrowseButton(const QString& text, QWidget* parent) : QPushButton(text, parent)
{
    connect(this, &AwBrowseButton::clicked, this, &AwBrowseButton::onClicked);
}

void AwBrowseButton::onClicked(bool checked)
{
    QString path = QFileDialog::getOpenFileName(this, tr("Find Files"), QDir::currentPath());
    if( ! path.isEmpty() )
    {
        Q_EMIT browsed(path);
    }
}

}
