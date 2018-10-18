#ifndef AUTOWARE_LAUNCHER_LAUNCH_BUTTON_H
#define AUTOWARE_LAUNCHER_LAUNCH_BUTTON_H

#include <QPushButton>

class QNetworkAccessManager;
class QNetworkReply;
class QLabel;
class QLineEdit;
class QPushButton;
class QGridLayout;

namespace autoware_rviz_plugins
{

class LaunchButton : public QPushButton
{
	Q_OBJECT
	
	public:

        LaunchButton( const QString& text, QWidget *parent = 0 );
        void addToLayout( QGridLayout* layout , int row );

        QString getFilePath() const;
        void setFilePath( const QString& path );

    public Q_SLOTS:

        void select();
        void launch();
        void launched( QNetworkReply* reply );

    private:

        QNetworkAccessManager* network;
        QLabel* label;
        QLineEdit* file;
        QPushButton* browse;
};

}

#endif
