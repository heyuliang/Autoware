#include "frames/single_file_selector.hpp"

#include "widgets/browse_button.hpp"

namespace autoware_launcher {

AwSingleFileSelector::AwSingleFileSelector(const QString& title)
{
    setFrameTitle(title);

    auto browse  = new AwBrowseButton;
    addButton(browse);

    content_ = new QLineEdit;
    content_->setReadOnly(true);
    addWidget(content_);

    connect(browse, &AwBrowseButton::fileBrowsed, this, &AwSingleFileSelector::onBrowsed);
}

void AwSingleFileSelector::onBrowsed(const QString& path)
{
    content_->setText(path);
}

}
