#ifndef AUTOWARE_LAUNCHER_MAIN_CONFIG_HPP_
#define AUTOWARE_LAUNCHER_MAIN_CONFIG_HPP_

#include <QJsonObject>

namespace autoware_launcher {

using AwConfig = QJsonObject;

class AwConfigInterface
{
    public:

        AwConfigInterface() = default;
        virtual ~AwConfigInterface() = default;

        virtual void saveConfig(AwConfig& config) const = 0;
        virtual void loadConfig(const AwConfig& config) = 0;
};

class AwConfigFile
{
    public:

        AwConfigFile() = default;
        virtual ~AwConfigFile() = default;

        bool save(const QString& path, const AwConfigInterface* tool);
        bool load(const QString& path,       AwConfigInterface* tool);

    private:

        AwConfig config_;
};

}

#endif // INCLUDE GUARD
