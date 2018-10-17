#ifndef AUTOWARE_LAUNCHER_H
#define AUTOWARE_LAUNCHER_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

namespace autoware_rviz_plugins
{

class AutowareLauncher : public rviz::Panel
{
	Q_OBJECT
	
	public:

		AutowareLauncher( QWidget* parent = 0 );

	private:

};

}

#endif
