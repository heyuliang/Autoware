import os
import subprocess


from python_qt_binding.QtCore import Signal as QtSignal
from python_qt_binding.QtCore import Slot   as QtSlot
from python_qt_binding.QtWidgets import QApplication as AwApplication
from python_qt_binding.QtWidgets import QSizePolicy
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QFileDialog
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QVBoxLayout

from profile import AwConfigNode

class AwMainFrame(QWidget):

    profile_selected = QtSignal(str)

    def __init__(self):

    	super(AwMainFrame, self).__init__()
        hlayout = QHBoxLayout()
        self.setLayout(hlayout)

        profile_label = QLabel("Profile")
        profile_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        hlayout.addWidget(profile_label)

        select_button = QPushButton("Select")
        select_button.clicked.connect(self.select_profile)
        hlayout.addWidget(select_button)

    def select_profile(self):
        path = QFileDialog.getExistingDirectory(self, "Select Config", os.path.expanduser("~/.autoware/profiles"))
        self.profile_selected.emit(path + "/")'

class AwMainWidget(QWidget):

    def __init__(self):
    	super(AwMainWidget, self).__init__()
        self.profile = None

        button_root = QPushButton("Launch")
        button_root.setCheckable(True)
        button_root.toggled.connect(self.launch_root)

        button_rviz = QPushButton("Rviz")
        button_rviz.setCheckable(True)
        button_rviz.toggled.connect(self.launch_rviz)

        hlayout = QHBoxLayout()
        hlayout.addStretch(0)
        hlayout.addWidget(button_root)
        hlayout.addWidget(button_rviz)

        frame_main = AwMainFrame()
        frame_main.profile_selected.connect(self.on_profile_selected)

        vlayout = QVBoxLayout()
        vlayout.addWidget(frame_main)
        vlayout.addWidget(AwMapFrame())
        vlayout.addLayout(hlayout)
        self.setLayout(vlayout)

    def launch_root(self, checked):
        if checked:
            if self.profile is not None:
                self.proc_root = subprocess.Popen(["roslaunch", "root.launch"], cwd = profile.rootpath)
        else:
                self.proc_root.terminate()

    def launch_rviz(self, checked):
        if checked:
            self.proc_rviz = subprocess.Popen(["rosrun", "rviz", "rviz"])
        else:
            self.proc_rviz.terminate()

    @QtSlot(str)
    def on_profile_selected(self, path):
        self.profile = AwConfigNode.load(path)
        self.profile.dump()
        

class AwMapFrame(QLabel):

    def __init__(self):

    	super(AwMapFrame, self).__init__()
        self.setText("Map Config")
        self.setStyleSheet("border: 1px solid;")

if __name__ == "__main__":
    path = "/home/isamu-takagi/AutowareWork/autoware-launcher-py/ros/src/util/packages/autoware_launcher/profiles/default/"
    profile.load(path).dump()