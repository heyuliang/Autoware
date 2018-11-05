#ifndef AUTOWARE_LAUNCHER_MULTI_FILE_SELECTOR_HPP_
#define AUTOWARE_LAUNCHER_MULTI_FILE_SELECTOR_HPP_

#include "base_widget.hpp"

#include <QTextEdit>

namespace autoware_launcher {

class AwMultiFileSelector : public AwBaseWidget
{
    Q_OBJECT

    public:

        AwMultiFileSelector(const QString& title);

    private:

        void onBrowsed(const QStringList& paths);

        QTextEdit* content_;
};

}

#endif // INCLUDE GUARD
