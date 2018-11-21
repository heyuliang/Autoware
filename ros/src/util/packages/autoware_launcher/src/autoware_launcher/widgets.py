from python_qt_binding.QtCore import QSettings
from python_qt_binding.QtWidgets import QApplication as AwApplication
from python_qt_binding.QtWidgets import QMainWindow

class AwMainWindow(QMainWindow):

    def __init__(self, widget_class):

        super(AwMainWindow, self).__init__()
        self.setCentralWidget(widget_class())
        
        settings = QSettings("Autoware", "AutowareLauncher")
        self.restoreGeometry(settings.value("geometry"))
 
    def closeEvent(self, event):

        settings = QSettings("Autoware", "AutowareLauncher")
        settings.setValue("geometry", self.saveGeometry())