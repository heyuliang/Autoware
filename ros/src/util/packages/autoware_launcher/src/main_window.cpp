#include "main_window.hpp"

// Temporary
#include <QGroupBox>
#include <QMenuBar>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QSettings>
#include "main_tool.hpp"
#include "main_config.hpp"

namespace autoware_launcher {

AwMainWindow::AwMainWindow()
{
    setWindowTitle("Autoware Launcher");

    auto main_menu = new QMenuBar;
    setMenuBar(main_menu);

    auto file_menu = main_menu->addMenu("File");
    auto load    = file_menu->addAction("Open Config");
    auto save    = file_menu->addAction("Save Config");
    auto save_as = file_menu->addAction("Save Config As");
    connect(load,    &QAction::triggered, this, &AwMainWindow::onLoad);
    connect(save,    &QAction::triggered, this, &AwMainWindow::onSave);
    connect(save_as, &QAction::triggered, this, &AwMainWindow::onSaveAs);

    auto widget = new AwMainTool;
    config_tool_ = widget;
    setCentralWidget(widget);
}


void AwMainWindow::onLoad()
{
    QString path = QFileDialog::getOpenFileName(this, tr("Load Config"), QDir::currentPath());
    if( path.isEmpty() )
    {
        return;
    }

    if( !config_file_.load(path, config_tool_) )
    {
        QMessageBox alert;
        alert.setText("Failed to load config");
        alert.exec();
    }
}

void AwMainWindow::onSave()
{

}

void AwMainWindow::onSaveAs()
{
    QString path = QFileDialog::getSaveFileName(this, tr("Save Config"), QDir::currentPath());
    if( path.isEmpty() )
    {
        return;
    }

    if( !config_file_.save(path, config_tool_) )
    {
        QMessageBox alert;
        alert.setText("Failed to save config");
        alert.exec();
    }
}


void AwMainWindow::closeEvent(QCloseEvent *event)
{
    saveSettings();
    QMainWindow::closeEvent(event);
}

void AwMainWindow::saveSettings()
{
    QSettings settings("Autoware", "AutowareLauncher");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void AwMainWindow::loadSettings()
{
    QSettings settings("Autoware", "AutowareLauncher");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
}


}
