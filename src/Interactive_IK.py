#!/usr/bin/env python3

import rospy
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import sys
from geometry_msgs import msg
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped, WrenchStamped, PointStamped
from std_msgs.msg import Bool, Float32,Int16,String

class MyPanel(QWidget):
    def __init__(self):
        super(MyPanel, self).__init__()

        # Create three buttons
        self.button1 = QPushButton("Run IK", self)
        self.button2 = QPushButton("Polyope: ON", self)
        self.button3 = QPushButton("Polytope: Off", self)
        self.button4 = QPushButton("Reset", self)


        # Create a vertical layout and add the buttons
        vbox = QVBoxLayout()
        vbox.addWidget(self.button1)
        vbox.addWidget(self.button2)
        vbox.addWidget(self.button3)

        # create the status bar
        self.progress_bar = QProgressBar(self)
        
        # add the status bar to the layout
        vbox.addWidget(self.progress_bar)

        vbox.addWidget(self.button4)

        
        # set the status message
        #progress_bar.showMessage('Ready')
        self.status_bar = QStatusBar(self)
        # set the status message

        # Set the layout for the panel
        self.setLayout(vbox)
        self.status_bar.showMessage('Ready')

        self.run_ik_bool = True
        self.poly_show = False
        self.ik_progress = 0.0
        self.ik_status = None
        self.ik_complete = True

        #self.poly_on = False
        # Connect the buttons to their respective callback functions
        self.button1.clicked.connect(self.button1_callback)
        self.button2.clicked.connect(self.button2_callback)
        self.button3.clicked.connect(self.button3_callback)
        self.button4.clicked.connect(self.button4_callback)


        # Create a publisher for the panel
        self.pub_start_ik = rospy.Publisher("run_ik", Bool, queue_size=1)
        self.pub_poly_display = rospy.Publisher("polytope_show", Bool, queue_size=1)
        self.sub_end_ik = rospy.Subscriber("ik_progress",Int16,self.completed_ik)
        self.sub_ik_status = rospy.Subscriber("status_ik",String,self.status_ik)



    def paintEvent(self, event):
        painter = QPainter(self)  # Create a QPainter object
        #painter.begin(self)  # Begin painting operations

        # Perform painting operations here, if needed

        #painter.end()  # End painting operations

    def status_ik(self,ik_status_msg):
        self.ik_status = ik_status_msg.data

        #if self.sub_ik_status.data:
        self.status_bar.showMessage(self.ik_status)
        self.button1.setEnabled(True)
        self.run_ik_bool = True
        self.ik_progress = 0
        self.ik_complete = True


    def completed_ik(self,ik_completed_msg):
        self.ik_progress = ik_completed_msg.data
        # Update the progress bar
        self.progress_bar.setValue(self.ik_progress)
        
              
        
    def button1_callback(self):
        # Create a message and publish it

        if self.ik_progress < 1:
            msg = Bool()
            msg.data = self.run_ik_bool
            self.run_ik_bool = False

            self.poly_show = msg.data
            self.pub_start_ik.publish(msg)

            #print('something pushed')
            # Update the status bar
            self.status_bar.showMessage("Running")            
            self.progress_bar.setValue(2)
            self.button1.setEnabled(False)  
            self.ik_complete = False      #else:
            
            

    def button2_callback(self):
        # Create a message and publish it
        msg = Bool()
        msg.data = True
        
        self.pub_poly_display.publish(msg)

        # Update the status bar
        self.status_bar.showMessage("Display")

        # Update the progress bar
        #self.progress_bar.setValue(50)

    def button3_callback(self):
        # Create a message and publish it
        msg = Bool()
        msg.data = False
        self.poly_off = msg.data
        self.pub_poly_display.publish(msg)

        # Update the status bar
        self.status_bar.showMessage("Hide")
        # Update the progress bar
        #self.progress_bar.setValue(75)


    def button4_callback(self):
        # Create a message and publish it
        #msg = Bool()
        #msg.data = False
        self.ik_progress = 0.0
        self.run_ik_bool = False
        self.button1.setEnabled(True)
        self.progress_bar.setValue(0)
        msg = Bool()
        msg.data = self.run_ik_bool
        

        self.poly_show = msg.data
        self.pub_start_ik.publish(msg)
        self.run_ik_bool = True


        #self.poly_off = msg.data
        #self.pub_poly_display.publish(msg)

        # Update the status bar
        self.status_bar.showMessage("Reset")
        # Update the progress bar
        #self.progress_bar.setValue(75)

def handle_exception(exc_type, exc_value, exc_traceback):
    # Handle and log the exception
    print(f"Exception occurred: {exc_type} - {exc_value}")
    # Perform any necessary cleanup or error handling here  

if __name__ == '__main__':
    
    
    # Set the exception hook to the custom handler
    sys.excepthook = handle_exception
    
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
    
