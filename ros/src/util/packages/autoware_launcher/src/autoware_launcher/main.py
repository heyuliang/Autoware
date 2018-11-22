import os

from python_qt_binding.QtCore import Signal as QtSignal
from python_qt_binding.QtCore import Slot   as QtSlot
from python_qt_binding.QtCore import QProcess
from python_qt_binding.QtGui import QPainter
from python_qt_binding.QtWidgets import QSizePolicy
from python_qt_binding.QtWidgets import QStyle
from python_qt_binding.QtWidgets import QStyleOption
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QFileDialog
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QVBoxLayout
from rospkg import RosPack
from config import AwConfigTree

class AwBasicFrameMenu(QWidget):

    def __init__(self, title = "[No Title]"):

    	super(AwBasicFrameMenu, self).__init__()
        self.mylayout = QHBoxLayout()
        self.setLayout(self.mylayout)

        label = QLabel(title)
        label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.mylayout.addWidget(label)

    def addMenuButton(self, button):

        self.mylayout.addWidget(button)

    def paintEvent(self, event):
        style_option = QStyleOption()
        style_option.initFrom(self)
        painter = QPainter(self)
        self.style().drawPrimitive(QStyle.PE_Widget, style_option, painter, self)



class AwBasicFrame(QWidget):

    def __init__(self):

    	super(AwBasicFrame, self).__init__()
        self.mylayout = QVBoxLayout()
        self.mylayout.setContentsMargins(0, 0, 0, 0)
        self.mylayout.setSpacing(0)
        self.setLayout(self.mylayout)

    def setFrameWidgets(self, menu, body):

        self.setObjectName("FrameWidget")
        menu.setObjectName("FrameMenu")
        body.setObjectName("FrameBody")
        self.setStyleSheet("#FrameWidget { border: 1px solid; } #FrameMenu { padding: 3px; border-bottom: 1px solid; } #FrameBody { padding: 3px; }")
        self.mylayout.addWidget(menu)
        self.mylayout.addWidget(body)
        
    def paintEvent(self, event):

        style_option = QStyleOption()
        style_option.initFrom(self)
        painter = QPainter(self)
        self.style().drawPrimitive(QStyle.PE_Widget, style_option, painter, self)



class AwMapConfigFrame(AwBasicFrame):

    def __init__(self):
    	super(AwMapConfigFrame, self).__init__()
        config_button = QPushButton("Config")
        config_button.setEnabled(False)
        menu = AwBasicFrameMenu("Map")
        menu.addMenuButton(config_button)
        self.body = QLabel("[No Data]")
        self.setFrameWidgets(menu, self.body)

    def loadConfig(self, config):
        self.body.setText(config.info["title"])

class AwSensorsConfigFrame(AwBasicFrame):

    def __init__(self):
    	super(AwSensorsConfigFrame, self).__init__()
        config_button = QPushButton("Config")
        config_button.setEnabled(False)
        menu = AwBasicFrameMenu("Sensors")
        menu.addMenuButton(config_button)
        self.body = QLabel("[No Data]")
        self.setFrameWidgets(menu, self.body)

    def loadConfig(self, config):
        self.body.setText(config.info["title"])

class AwRvizConfigFrame(AwBasicFrame):

    def __init__(self):
    	super(AwRvizConfigFrame, self).__init__()
        config_button = QPushButton("Config")
        config_button.setEnabled(False)
        menu = AwBasicFrameMenu("Rviz")
        menu.addMenuButton(config_button)
        self.body = QLabel("[No Data]")
        self.setFrameWidgets(menu, self.body)

    def loadConfig(self, config):
        self.body.setText(config.info["title"])



class AwRootConfigFrame(AwBasicFrame):

    profileSelected = QtSignal(AwConfigTree)

    def __init__(self):

    	super(AwRootConfigFrame, self).__init__()

        select_button = QPushButton("Select")
        select_button.clicked.connect(self.selectProfile)

        menu = AwBasicFrameMenu("Profile")
        menu.addMenuButton(select_button)

        self.body = QLabel("[No Data]")
        
        self.setFrameWidgets(menu, self.body)

    def selectProfile(self):
        default_path = os.path.join( RosPack().get_path("autoware_launcher"), "profiles")
        path = QFileDialog.getExistingDirectory(self, "Select Profile", default_path)
        if path:
            self.profile = AwConfigTree(path)
            self.profile.load()
            self.profileSelected.emit(self.profile)

    def loadConfig(self, config):
        self.body.setText(config.getTreePath())



class AwRootConfigWidget(QWidget):

    def __init__(self):
    	super(AwRootConfigWidget, self).__init__()
        self.profile = None
        self.frames = {}

        button_root = QPushButton("Launch")
        button_root.setCheckable(True)
        button_root.toggled.connect(self.launch_root)

        button_rviz = QPushButton("Rviz")
        button_rviz.setEnabled(False)
        button_rviz.setCheckable(True)
        button_rviz.toggled.connect(self.launch_rviz)

        hlayout = QHBoxLayout()
        hlayout.addStretch(0)
        hlayout.addWidget(button_root)
        hlayout.addWidget(button_rviz)

        self.frames["root"] = AwRootConfigFrame()
        self.frames["root"] .profileSelected.connect(self.loadConfig)

        self.frames["map"]     = AwMapConfigFrame()
        self.frames["sensors"] = AwSensorsConfigFrame()
        self.frames["rviz"]    = AwRvizConfigFrame()

        vlayout = QVBoxLayout()
        vlayout.addWidget(self.frames["root"])
        vlayout.addWidget(self.frames["map"])
        vlayout.addWidget(self.frames["sensors"])
        vlayout.addWidget(self.frames["rviz"])
        vlayout.addStretch()
        vlayout.addLayout(hlayout)
        self.setLayout(vlayout)

    def launch_root(self, checked):
        if self.profile is not None:
            if checked:
                proc = os.path.join(self.profile.getTreePath(), "root.launch")
                args = " ROOT_PATH:=" + self.profile.getTreePath()
                self.proc_root = QProcess()
                print "roslaunch " + proc + args
                self.proc_root.start("roslaunch " + proc + args)
            else:
                self.proc_root.terminate()

    def launch_rviz(self, checked):
        if checked:
            self.proc_rviz = QProcess()
            self.proc_rviz.start("roslaunch /home/isamu-takagi/rviz.launch")
        else:
            self.proc_rviz.terminate()

    @QtSlot(AwConfigTree)
    def loadConfig(self, profile):

        #self
        self.profile = profile
        self.frames["root"].loadConfig(profile)

        #children
        for child in self.profile.getRootNode().children:
            self.frames[child.getNodeName()].loadConfig(child)

if __name__ == "__main__":
    pass