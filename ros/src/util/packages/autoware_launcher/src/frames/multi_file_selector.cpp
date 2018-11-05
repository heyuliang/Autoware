#include "frames/multi_file_selector.hpp"

#include "widgets/browse_button.hpp"

namespace autoware_launcher {

AwMultiFileSelector::AwMultiFileSelector(const QString& title)
{
    setFrameTitle(title);

    auto browse  = new AwBrowseButton;
    browse->setBrowseTarget(AwBrowseButton::BrowseTarget::FileList);
    addButton(browse);

    content_ = new QTextEdit;
    content_->setReadOnly(true);
    content_->setLineWrapMode(QTextEdit::NoWrap);
    addWidget(content_);

    connect(browse, &AwBrowseButton::listBrowsed, this, &AwMultiFileSelector::onBrowsed);
}

void AwMultiFileSelector::onBrowsed(const QStringList& paths)
{
    content_->setText(paths.join('\n'));
}

}
