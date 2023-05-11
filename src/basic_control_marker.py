#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from visualization_msgs.msg import Marker
from interactive_markers.interactive_marker_server import *

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from math import radians

def makeBox(msg):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.25
    marker.scale.y = msg.scale * 0.25
    marker.scale.z = msg.scale * 0.25
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.2
    marker.color.a = 1.0

    return marker

def make6DofMarker(pose):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "simple_6dof"
    int_marker.description = "Simple 6-DOF Control"

    # insert a box
    box_marker = makeBox(int_marker)
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append(box_marker)
    int_marker.controls.append(box_control)

    # create 6DOF control
    control = InteractiveMarkerControl()
    control.orientation_mode = InteractiveMarkerControl.INHERIT
    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
    control.always_visible = True
    int_marker.controls.append(control)

    int_marker.pose = pose
    return int_marker

def markerFeedback(feedback):
    p = feedback.pose.position
    o = feedback.pose.orientation
    rospy.loginfo(feedback.marker_name + " is now at x:" + str(p.x) + ", y:" + str(p.y) + ", z:" + str(p.z) + " with quaternion (x,y,z,w): " + str(o.x) + ", " + str(o.y) + ", " + str(o.z) + ", " + str(o.w))

if __name__ == '__main__':
    rospy.init_node("six_dof_marker")
    rate = rospy.Rate(250)

    pub = rospy.Publisher('simple_marker_pose', PoseStamped, queue_size=10)

    int_marker_pub = rospy.Publisher("simple_6dof", InteractiveMarker, queue_size=10)

    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("simple_marker")

    pose = Pose()
    pose.position.x = 0.5
    pose.position.y = 0.5
    pose.position.z = 0.5
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    int_marker = make6DofMarker(pose)

    server.insert(int_marker, markerFeedback)

    server.applyChanges()

    while not rospy.is_shutdown():
        int_marker_pub.publish(int_marker)
        pose_stamped = PoseStamped()
        pose_stamped.pose = int_marker.pose
        pose_stamped.header.frame_id = int_marker.header.frame_id
        pub.publish(pose_stamped)
        rate.sleep()

    server.clear()
    server.applyChanges()
