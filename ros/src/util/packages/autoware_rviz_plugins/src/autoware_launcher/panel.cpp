#include "panel.h"
#include "launch_button.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QPushButton>

namespace autoware_rviz_plugins
{

AutowareLauncher::AutowareLauncher(QWidget* parent) : rviz::Panel(parent)
{
	QPushButton* engage = new QPushButton("Engage");
	engage->setCheckable(true);
	engage->setEnabled(false);
	engage->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

	auto launcher = new QGridLayout;
	launcher->addWidget( new LaunchButton("Localization") , 0, 0 );
	launcher->addWidget( new LaunchButton("Detection")    , 0, 1 );
	launcher->addWidget( new LaunchButton("Prediction")   , 0, 2 );
	launcher->addWidget( new LaunchButton("Decision")     , 1, 0 );
	launcher->addWidget( new LaunchButton("Mission")      , 1, 1 );
	launcher->addWidget( new LaunchButton("Motion")       , 1, 2 );

	auto command = new QGridLayout;
	command->addWidget( new QPushButton("GO")                 , 0, 0 );
	command->addWidget( new QPushButton("STOP")               , 0, 1 );
	command->addWidget( new QPushButton("EMERGENCY")          , 0, 2 );
	command->addWidget( new QPushButton("OBSTACLE\nAVOIDANCE"), 1, 0 );
	command->addWidget( new QPushButton("LANE\nCHANGE")       , 1, 1 );
	command->addWidget( new QPushButton("LANE\nRETURN")       , 1, 2 );

	QHBoxLayout* layout = new QHBoxLayout;
	layout->addLayout( launcher, 3 );
	layout->addWidget( engage,   1 );
	layout->addLayout( command,  3 );
	setLayout( layout );
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::AutowareLauncher, rviz::Panel)
