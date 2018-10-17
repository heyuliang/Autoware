#ifndef AUTOWARE_LAUNCHER_LAUNCH_BUTTON_H
#define AUTOWARE_LAUNCHER_LAUNCH_BUTTON_H

#include <QPushButton>

namespace autoware_rviz_plugins
{

class LaunchButton : public QPushButton
{
	Q_OBJECT
	
	public:

		LaunchButton( const QString& text, QWidget *parent = 0 );

	private Q_SLOTS:

		void pushed();

};

}

#endif
