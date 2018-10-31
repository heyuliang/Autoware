#ifndef AUTOWARE_LAUNCHER_MAP_CONFIG_WIDGET_HPP_
#define AUTOWARE_LAUNCHER_MAP_CONFIG_WIDGET_HPP_

#include "base_widget.hpp"

namespace autoware_launcher {

class AwMapConfigWidget : public AwBaseWidget
{
    Q_OBJECT

    public:

        AwMapConfigWidget();

    public Q_SLOTS:

        void test();
};

}

#endif // INCLUDE GUARD
