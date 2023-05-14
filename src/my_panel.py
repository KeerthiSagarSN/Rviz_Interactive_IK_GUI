#!/usr/bin/env python3

import rospy
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import sys

class MyPanel(QWidget):
    def __init__(self):
        super(MyPanel, self).__init__()

        # Create three buttons
        button1 = QPushButton("Run IK", self)
        button2 = QPushButton("Polyope: ON", self)
        button3 = QPushButton("Polytope: Off", self)

        # Create a vertical layout and add the buttons
        vbox = QVBoxLayout()
        vbox.addWidget(button1)
        vbox.addWidget(button2)
        vbox.addWidget(button3)

        # Set the layout for the panel
        self.setLayout(vbox)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('my_rviz_panel')

    # Create a QApplication instance
    app = QApplication(sys.argv)

    # Create the panel widget
    panel = MyPanel()

    # Show the panel widget
    panel.show()

    # Run the Qt event loop
    sys.exit(app.exec_())
