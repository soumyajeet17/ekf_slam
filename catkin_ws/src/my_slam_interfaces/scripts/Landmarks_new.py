#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from my_slam_interfaces.msg import LandmarkArray

def callback(msg):
    markers = MarkerArray()
    for i, lm in enumerate(msg.landmarks):
        marker = Marker()
        marker.header = msg.header
        marker.ns = "landmarks"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = lm.x
        marker.pose.position.y = lm.y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.1  # sphere diameter
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        markers.markers.append(marker)
    pub.publish(markers)

rospy.init_node("landmark_visualizer")
pub = rospy.Publisher("landmark_markers", MarkerArray, queue_size=10)
sub = rospy.Subscriber("landmarks", LandmarkArray, callback)
rospy.spin()

