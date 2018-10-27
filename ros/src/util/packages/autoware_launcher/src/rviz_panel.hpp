#ifndef AUTOWARE_LAUNCHER_RVIZ_PANEL_HPP_
#define AUTOWARE_LAUNCHER_RVIZ_PANEL_HPP_

#include <rviz/panel.h>

namespace autoware_launcher {

class RvizPanel : public rviz::Panel
{
    Q_OBJECT

    public:

        RvizPanel();
        virtual ~RvizPanel() = default;

        void save(rviz::Config config) const override;
        void load(const rviz::Config& config) override;
};

}

#endif // INCLUDE GUARD
