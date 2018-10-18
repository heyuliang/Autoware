#ifndef AUTOWARE_LAUNCHER_COMMAND_BUTTON_H
#define AUTOWARE_LAUNCHER_COMMAND_BUTTON_H

#include <QPushButton>

namespace autoware_rviz_plugins
{

class CommandButton : public QPushButton
{
	Q_OBJECT
	
	public:

        CommandButton( const QString& text, QWidget *parent = 0 );

	private Q_SLOTS:

		void pushed();

};

}

#endif
