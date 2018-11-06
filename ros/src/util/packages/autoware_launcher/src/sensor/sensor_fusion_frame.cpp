#include "sensor_fusion_frame.hpp"
#include "sensor_fusion_window.hpp"

#include "frame_title.hpp"

#include <QPushButton>
#include <QDebug>

namespace autoware_launcher {

AwSensorFusionFrame::AwSensorFusionFrame()
{
    auto config_button = new QPushButton("Config");
    auto launch_button = new QPushButton("Launch");

    auto frame_title = new AwFrameTitle;
    frame_title->addTitleButton(config_button);
    frame_title->addTitleButton(launch_button);
    addInnerLayout(frame_title);

    connect(config_button, &QPushButton::clicked, [this]()
    {
        auto config_window = new AwSensorFusionWindow;
        config_window->resize(0.8 * window()->size());
        config_window->move(window()->pos() + window()->rect().center() - config_window->rect().center());
        config_window->setAttribute(Qt::WA_DeleteOnClose);
        config_window->setWindowModality(Qt::ApplicationModal);
        config_window->show();
    });
}

}
