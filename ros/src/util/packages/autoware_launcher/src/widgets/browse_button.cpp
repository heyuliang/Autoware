#include "browse_button.hpp"

#include <QFileDialog>

namespace autoware_launcher {

AwBrowseButton::AwBrowseButton(const QString& text, QWidget* parent) : QPushButton(text, parent)
{
    target_ = BrowseTarget::File;
    connect(this, &AwBrowseButton::clicked, this, &AwBrowseButton::onClicked);
}

void AwBrowseButton::setBrowseTarget(BrowseTarget target)
{
    target_ = target;
}

void AwBrowseButton::onClicked(bool checked)
{
    if(target_ == BrowseTarget::File)
    {
        QString path = QFileDialog::getOpenFileName(this, tr("Find File"), QDir::currentPath());
        if( ! path.isEmpty() )
        {
            Q_EMIT fileBrowsed(path);
        }
        return;
    }

    if(target_ == BrowseTarget::FileList)
    {
        QStringList paths = QFileDialog::getOpenFileNames(this, tr("Find Files"), QDir::currentPath());
        if( ! paths.isEmpty() )
        {
            Q_EMIT listBrowsed(paths);
        }
        return;
    }

    if(target_ == BrowseTarget::SaveFile)
    {
        QString path = QFileDialog::getSaveFileName(this, tr("Save File"), QDir::currentPath());
        if( ! path.isEmpty() )
        {
            Q_EMIT fileBrowsed(path);
        }
        return;
    }

    if(target_ == BrowseTarget::Direcroty)
    {
        QString path = QFileDialog::getExistingDirectory(this, tr("Find Directory"), QDir::currentPath());
        if( ! path.isEmpty() )
        {
            Q_EMIT fileBrowsed(path);
        }
        return;
    }
}

}
