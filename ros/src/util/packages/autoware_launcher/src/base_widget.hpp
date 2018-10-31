#ifndef AUTOWARE_LAUNCHER_BASE_WIDGET_HPP_
#define AUTOWARE_LAUNCHER_BASE_WIDGET_HPP_

#include <QWidget>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace autoware_launcher {

class AwBaseWidget : public QWidget
{
    Q_OBJECT

    public:

        AwBaseWidget();
        void addButton(QWidget* button);

    protected:

        void paintEvent(QPaintEvent* event) override;

        QHBoxLayout* header_;
        QLabel* title_;
        QLabel* value_;
        QWidget* content_;
};

}

#endif // INCLUDE GUARD
