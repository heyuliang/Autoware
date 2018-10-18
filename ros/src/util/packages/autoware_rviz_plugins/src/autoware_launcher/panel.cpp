// Internal headers
#include "panel.h"
#include "launch_button.h"
#include "command_button.h"

// Autoware headers
#include "diag_lib/diag_filter.h"

// Qt headers
#include <QHBoxLayout>
#include <QGridLayout>
#include <QTabWidget>
#include <QPlainTextEdit>

namespace autoware_rviz_plugins
{

AutowareLauncher::AutowareLauncher(QWidget* parent) : rviz::Panel(parent)
{
    const std::pair<QString, QString> data[LAUNCHER_COUNT] =
    {
        {"Localization", "/home/isamu-takagi/Autoware/docs/quick_start/my_localization.launch"    },
        {"Detection",    "/home/isamu-takagi/Autoware/docs/quick_start/my_detection.launch"       },
        {"Prediction",   "/home/isamu-takagi/Autoware/docs/quick_start/my_map.launch"             },
        {"Decision",     "/home/isamu-takagi/Autoware/docs/quick_start/my_sensing.launch"         },
        {"Mission",      "/home/isamu-takagi/Autoware/docs/quick_start/my_mission_planning.launch"},
        {"Motion",       "/home/isamu-takagi/Autoware/docs/quick_start/my_motion_planning.launch" }
    };

    // Launch button and file selector
    auto launch = new QGridLayout;
    auto select = new QGridLayout;
    for(int i = 0; i < LAUNCHER_COUNT; ++i)
    {
        launchers[i] = new LaunchButton(data[i].first);
        launchers[i]->addToLayout( select, i );
        launch->addWidget( launchers[i], i/3, i%3 );
    }

    // Command button group
	auto command = new QGridLayout;
    command->addWidget( new CommandButton("GO")                 , 0, 0 );
    command->addWidget( new CommandButton("STOP")               , 0, 1 );
    command->addWidget( new CommandButton("EMERGENCY")          , 0, 2 );
    command->addWidget( new CommandButton("OBSTACLE\nAVOIDANCE"), 1, 0 );
    command->addWidget( new CommandButton("LANE\nCHANGE")       , 1, 1 );
    command->addWidget( new CommandButton("LANE\nRETURN")       , 1, 2 );

    // Engage button
    QPushButton* engage = new QPushButton("Engage");
    engage->setCheckable(true);
    engage->setEnabled(false);
    engage->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

    // Widget for launch/command
    auto main_layout = new QHBoxLayout;
    auto main_widget = new QWidget;
    main_layout->addLayout( launch,  3 );
    main_layout->addWidget( engage,  1 );
    main_layout->addLayout( command, 3 );
    main_widget->setLayout( main_layout );

    // Widget for file select
    auto file_widget = new QWidget;
    file_widget->setLayout( select );

    // Widget for message text
    auto text_widget = new QPlainTextEdit;
    text_widget->setReadOnly(true);

    // Tab widget with dummy layout
    auto tab_layout = new QGridLayout;
    auto tab_widget = new QTabWidget;
    tab_widget->addTab( main_widget, "Launcher" );
    tab_widget->addTab( file_widget, "Settings" );
    tab_widget->addTab( text_widget, "Messages" );
    tab_layout->addWidget( tab_widget );
    setLayout( tab_layout );
}

void AutowareLauncher::save( rviz::Config config ) const
{
    // Save launcher filepath
    rviz::Panel::save( config );
    for(int i = 0; i < LAUNCHER_COUNT; ++i)
    {
        QString key = "File" + QString::number(i);
        config.mapSetValue( key, launchers[i]->getFilePath() );
    }
}

void AutowareLauncher::load( const rviz::Config& config )
{
    // Load launcher filepath
    rviz::Panel::load( config );
    for(int i = 0; i < LAUNCHER_COUNT; ++i)
    {
        QString key = "File" + QString::number(i);
        QString path;
        if( config.mapGetString( key, &path ) )
        {
            launchers[i]->setFilePath( path );
        }
    }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::AutowareLauncher, rviz::Panel)
