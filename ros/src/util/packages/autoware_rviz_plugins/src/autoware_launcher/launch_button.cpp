#include "launch_button.h"

#include <QFileDialog>
#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkRequest>
#include <QtNetwork/QNetworkReply>
#include <QJsonObject>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QGridLayout>

namespace autoware_rviz_plugins
{

LaunchButton::LaunchButton(const QString& text, QWidget* parent ) : QPushButton(text, parent)
{
    network = new QNetworkAccessManager(this);
    label   = new QLabel(text, this);
    file    = new QLineEdit(this);
    browse  = new QPushButton("Browse", this);

    connect(browse,  &QPushButton::released, this, &LaunchButton::select);
    connect(this,    &QPushButton::released, this, &LaunchButton::launch);
    connect(network, &QNetworkAccessManager::finished, this, &LaunchButton::launched);

    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
}

QString LaunchButton::getFilePath() const
{
    return file->text();
}

void LaunchButton::setFilePath( const QString& path )
{
    file->setText( path );
}

void LaunchButton::select()
{
    QString path = QFileDialog::getOpenFileName(this, tr("Find Files"), QDir::currentPath());
    if( ! path.isEmpty() )
    {
        file->setText(path);
    }
}

void LaunchButton::addToLayout( QGridLayout* layout, int row )
{
    layout->addWidget( label,  row, 0 );
    layout->addWidget( browse, row, 1 );
    layout->addWidget( file,   row, 2 );
}

void LaunchButton::launch()
{
    QString path = file->text();
    if( ! path.isEmpty() )
    {
        QNetworkRequest request( QUrl("http://localhost:8080/roslaunch") );
        request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

        QJsonObject json;
        json.insert("package", "");
        json.insert("launch_filename", path);

        network->post(request, QJsonDocument(json).toJson());
    }
}

void LaunchButton::launched( QNetworkReply* reply )
{
    if(reply->error() == QNetworkReply::NoError)
    {
        setStyleSheet("background-color: #00CC00");
    }
    else
    {
        setStyleSheet("background-color: #CC0000");
    }
}

}
