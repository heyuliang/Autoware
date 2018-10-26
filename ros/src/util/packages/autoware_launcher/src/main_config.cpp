#include "main_config.hpp"

#include <QFile>
#include <QJsonDocument>

namespace autoware_launcher {

bool AwConfigFile::save(const QString& path, const AwConfigInterface *tool)
{
    QFile file(path);
    if( !file.open(QIODevice::WriteOnly) )
    {
        return false;
    }
    tool->saveConfig(config_);
    file.write(QJsonDocument(config_).toJson());
    return true;
}

bool AwConfigFile::load(const QString& path, AwConfigInterface* tool)
{
    QFile file(path);
    if( !file.open(QIODevice::ReadOnly) )
    {
        return false;
    }
    config_ = QJsonDocument::fromJson(file.readAll()).object();
    tool->loadConfig(config_);
    return true;
}

}
