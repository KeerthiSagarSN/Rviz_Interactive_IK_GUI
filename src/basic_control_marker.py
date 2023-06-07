#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import *

class SphereMarker:
    def __init__(self):
        self.server = InteractiveMarkerServer("sphere_marker_server")
        self.create_marker()
        self.pub_sphere_marker = rospy.Publisher("interactive_sphere", Pose, queue_size=100)
        self.sphere_pos = Pose()

    def create_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"
        int_marker.pose.position = Point(0.5, 0.5, 0.5)
        int_marker.scale = 0.1

        int_marker.name = "sphere_marker"
        int_marker.description = "Interactive Sphere Marker"

        # Create a control that contains the sphere mesh
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        control.markers.append(self.make_sphere_marker())
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.processFeedback)
        self.server.applyChanges()

    def make_sphere_marker(self):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 1.0*0.08
        marker.scale.y = 1.0*0.08
        marker.scale.z = 1.0*0.08
        marker.color = ColorRGBA(0.0, 0.0, 0.0, 255.0)
        return marker

    def processFeedback(self, feedback):
        p = feedback.pose.position
        self.sphere_pos.position = p
        self.pub_sphere_marker.publish(self.sphere_pos)
        #rospy.loginfo("Sphere Marker is now at %f, %f, %f" % (p.x, p.y, p.z))


try:
    rospy.init_node("sphere_marker_node")
    sphere_marker = SphereMarker()
    rospy.spin()

except Exception as e:
    print('Exception:', e)
