#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import *
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from tf.transformations import quaternion_from_euler
import math

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

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # Create controls for 6-DOF manipulation
        ## Enable this for quaternion - for now I am not going to use this
        # for axis in [(1, 0, 0), (0, 1, 0), (0, 0, 1)]:
        #     for mode, name in [(InteractiveMarkerControl.ROTATE_AXIS, "rotate"), 
        #                     (InteractiveMarkerControl.MOVE_AXIS, "move")]:
        #         control = InteractiveMarkerControl()
        #         quat = quaternion_from_euler(axis[0] * math.pi/2, axis[1] * math.pi/2, axis[2] * math.pi/2)
        #         quat = self.normalize_quaternion(quat)
        #         control.orientation.x, control.orientation.y, control.orientation.z, control.orientation.w = quat
        #         control.name = f"{name}_{['x', 'y', 'z'][axis.index(1)]}"
        #         control.interaction_mode = mode
        #         int_marker.controls.append(control)

        self.server.insert(int_marker, self.processFeedback)
        self.server.applyChanges()
    def normalize_quaternion(self,quaternion):
        norm = math.sqrt(sum(x*x for x in quaternion))
        if norm == 0:
            return (0, 0, 0, 1)
        return tuple(x/norm for x in quaternion)
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
        self.sphere_pos.orientation.x = feedback.pose.orientation.x
        self.sphere_pos.orientation.y = feedback.pose.orientation.y
        self.sphere_pos.orientation.z = feedback.pose.orientation.z
        self.sphere_pos.orientation.w = feedback.pose.orientation.w
        self.pub_sphere_marker.publish(self.sphere_pos)
        #rospy.loginfo("Sphere Marker is now at %f, %f, %f" % (p.x, p.y, p.z))


try:
    rospy.init_node("sphere_marker_node")
    sphere_marker = SphereMarker()
    rospy.spin()

except Exception as e:
    print('Exception:', e)
