#include "launch_button.h"

namespace autoware_rviz_plugins
{

LaunchButton::LaunchButton( const QString& text, QWidget* parent ) : QPushButton(text, parent)
{
	setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

	connect(this, &LaunchButton::released, this, &LaunchButton::pushed);
}

void LaunchButton::pushed()
{
	setStyleSheet("background-color: #00CC00");
}

}
