import subprocess

from python_qt_binding.QtWidgets import QApplication as AwApplication
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QVBoxLayout

class AwMainWidget(QWidget):

    def __init__(self):
    	super(AwMainWidget, self).__init__()

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

        vlayout = QVBoxLayout()
        vlayout.addWidget(AwMapFrame())
        vlayout.addLayout(hlayout)
        self.setLayout(vlayout)


    def launch_root(self, checked):
        if checked:
            profiles = "/home/isamu-takagi/AutowareWork/autoware-launcher-py/ros/src/util/packages/autoware_launcher/profiles/"
            self.proc_root = subprocess.Popen(["roslaunch", "root.launch"], cwd = profiles + "default")
        else:
            self.proc_root.terminate()

    def launch_rviz(self, checked):
        if checked:
            self.proc_rviz = subprocess.Popen(["rosrun", "rviz", "rviz"])
        else:
            self.proc_rviz.terminate()


class AwMapFrame(QLabel):

    def __init__(self):

    	super(AwMapFrame, self).__init__()
        self.setText("Map Config")
        self.setStyleSheet("border: 1px solid;")
