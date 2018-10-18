#ifndef AUTOWARE_LAUNCHER_H
#define AUTOWARE_LAUNCHER_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

namespace autoware_rviz_plugins
{

class LaunchButton;

class AutowareLauncher : public rviz::Panel
{
	Q_OBJECT
	
	public:

        AutowareLauncher( QWidget* parent = 0 );

        void save( rviz::Config config ) const override;
        void load( const rviz::Config& config ) override;

    private:

        static constexpr int LAUNCHER_COUNT = 6;
        LaunchButton* launchers[LAUNCHER_COUNT];
};

}

#endif
