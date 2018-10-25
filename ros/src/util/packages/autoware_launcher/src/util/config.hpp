#ifndef AUTOWARE_LAUNCHER_UTIL_CONFIG_HPP_
#define AUTOWARE_LAUNCHER_UTIL_CONFIG_HPP_

#include <QJsonObject>

// Temporary Code
#include <QFile>
#include <QJsonDocument>

namespace autoware_launcher {

class AwConfig : public QJsonObject
{
    public:

        AwConfig() = default;
        virtual ~AwConfig() = default;

        bool save()
        {
            QFile saveFile(QStringLiteral("~/work/save.json"));
            if (saveFile.open(QIODevice::WriteOnly))
            {
                saveFile.write(QJsonDocument(*this).toJson());
            }
            else
            {
                // Temporary Code
                printf("open error");
            }
        }

    private:


};

class AwConfigMixin
{
    public:

        AwConfigMixin() = default;
        virtual ~AwConfigMixin() = default;

        virtual void saveConfig(AwConfig& config) = 0;
        virtual void loadConfig(AwConfig& config) = 0;
};

}

#endif // INCLUDE GUARD
