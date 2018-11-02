#ifndef AUTOWARE_LAUNCHER_MAIN_TOOL_HPP_
#define AUTOWARE_LAUNCHER_MAIN_TOOL_HPP_

#include "base_window.hpp"
#include "main_config.hpp"
#include "browse_launcher.hpp"
#include <QWidget>

namespace autoware_launcher {

class AwMainTool final : public AwBaseWindow, public AwConfigInterface
{
    Q_OBJECT

    public:

        AwMainTool(QWidget* parent = nullptr);
        virtual ~AwMainTool() = default;

        void saveConfig(AwConfig &config) const override;
        void loadConfig(const AwConfig &config) override;

    private:

        QMap<QString, AwBrowseLauncher*> children;
};

}

#endif // INCLUDE GUARD
