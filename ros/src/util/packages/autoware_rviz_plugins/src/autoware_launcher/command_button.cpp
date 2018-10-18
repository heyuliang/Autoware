#include "command_button.h"

namespace autoware_rviz_plugins
{

CommandButton::CommandButton( const QString& text, QWidget* parent ) : QPushButton(text, parent)
{
    setEnabled(false);
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
}

void CommandButton::pushed()
{

}

}
