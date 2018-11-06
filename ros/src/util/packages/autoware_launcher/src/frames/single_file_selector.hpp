#ifndef AUTOWARE_LAUNCHER_SINGLE_FILE_SELECTOR_HPP_
#define AUTOWARE_LAUNCHER_SINGLE_FILE_SELECTOR_HPP_

#include "base_widget.hpp"

#include <QLineEdit>

namespace autoware_launcher {

class AwSingleFileSelector : public AwBaseWidget
{
    Q_OBJECT

    public:

        AwSingleFileSelector(const QString& title);

    private Q_SLOTS:

        void onBrowsed(const QString& path);

    private:

        QLineEdit* content_;
};

}

#endif // INCLUDE GUARD
