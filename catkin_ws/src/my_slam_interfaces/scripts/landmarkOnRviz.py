#!/usr/bin/env python3
import rospy
from my_slam_interfaces.msg import LandmarkArray
from visualization_msgs.msg import Marker, MarkerArray

def callback(msg):
    marker_array = MarkerArray()
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
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Sphere diameter (10cm)
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker_array.markers.append(marker)

    pub.publish(marker_array)

rospy.init_node("landmarks_to_markers")
sub = rospy.Subscriber("/landmarks", LandmarkArray, callback)
pub = rospy.Publisher("/landmark_markers", MarkerArray, queue_size=10)

rospy.spin()

