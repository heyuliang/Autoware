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
        void addWidget(QWidget* widget);

        void setFrameTitle(const QString& title);

    protected:

        void paintEvent(QPaintEvent* event) override;

        QLabel* title_;
        QHBoxLayout* hlayout_;
        QVBoxLayout* vlayout_;

    private:

        static constexpr int layout_margin = 4;
};

}

#endif // INCLUDE GUARD
