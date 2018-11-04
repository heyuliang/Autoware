#include "rviz_panel.hpp"

#include "widgets/browse_button.hpp"
#include "launch_button.hpp"
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QTabWidget>
#include <QGridLayout>

namespace autoware_launcher {

// Temporary
constexpr int LAUNCHER_COUNT = 6;
QLineEdit* launch_path[LAUNCHER_COUNT];
QString launch_text[LAUNCHER_COUNT] = { "Localization", "Detection", "Prediction", "Decision", "Mission", "Motion"};

// Temporary
constexpr int STATE_COUNT = 6;
QString state_text[LAUNCHER_COUNT] = { "GO", "STOP", "EMERGENCY", "OBSTACLE\nAVOIDANCE", "LANE\nCHANGE", "LANE\nRETURN"};

// Temporary
void createLauncher(QGridLayout* main, QGridLayout* conf)
{
    for(int i = 0; i < LAUNCHER_COUNT; ++i)
    {
        auto launch = new AwLaunchButton(launch_text[i]);
        launch->setProgram("roslaunch");
        launch->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        main->addWidget(launch, i/3, i%3);

        auto text   = new QLabel(launch_text[i]);
        auto path   = new QLineEdit;
        auto browse = new AwBrowseButton("Browse");
        conf->addWidget(text,   i, 0);
        conf->addWidget(path,   i, 1);
        conf->addWidget(browse, i, 2);

        launch_path[i] = path;
        QObject::connect(browse, &AwBrowseButton::fileBrowsed, path, &QLineEdit::setText);
        QObject::connect(path, &QLineEdit::textChanged, launch, &AwLaunchButton::setNativeArguments);
    }
}

// Temporary
void createState(QGridLayout* main)
{
    for(int i = 0; i < STATE_COUNT; ++i)
    {
        auto button = new QPushButton(state_text[i]);
        button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        main->addWidget(button, i/3, i%3 + 4);
    }
}

// Temporary
void createEngage(QGridLayout* main)
{
    auto button = new QPushButton("Engage");
    button->setStyleSheet("font-size: 32px");
    button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    main->addWidget(button, 0, 3, 2, 1);
}

RvizPanel::RvizPanel()
{
    auto main_layout = new QGridLayout;
    auto conf_layout = new QGridLayout;
    createLauncher(main_layout, conf_layout);
    createState(main_layout);
    createEngage(main_layout);

    auto main_widget = new QWidget;
    auto conf_widget = new QWidget;
    main_widget->setLayout(main_layout);
    conf_widget->setLayout(conf_layout);

    auto layout = new QGridLayout;
    auto widget = new QTabWidget;
    widget->addTab(main_widget, "Launcher");
    widget->addTab(conf_widget, "Settings");
    layout->addWidget(widget);
    setLayout(layout);
}

void RvizPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    for(int i = 0; i < LAUNCHER_COUNT; ++i)
    {
        config.mapSetValue("Launch"+QString::number(i), launch_path[i]->text());
    }
}

void RvizPanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
    for(int i = 0; i < LAUNCHER_COUNT; ++i)
    {
        QString path;
        if(config.mapGetString("Launch"+QString::number(i), &path))
        {
            launch_path[i]->setText(path);
        }
    }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_launcher::RvizPanel, rviz::Panel)
