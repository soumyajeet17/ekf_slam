#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from my_slam_interfaces.msg import LandmarkArray
from nav_msgs.msg import Odometry

class LandmarkVisualizer:
    def __init__(self):
        rospy.init_node("landmark_visualizer")

        # Publishers
        self.landmark_pub = rospy.Publisher("/landmark_markers", MarkerArray, queue_size=10)
        self.odom_pub = rospy.Publisher("/odom_marker", Marker, queue_size=10)

        # Subscribers
        rospy.Subscriber("/landmarks", LandmarkArray, self.landmark_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.frame_id = "map"   # RViz fixed frame

    def landmark_callback(self, msg):
        markers = MarkerArray()

        for i, lm in enumerate(msg.landmarks):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "landmarks"
            marker.id = lm.id if lm.id != 0 else i   # use ID if available
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = lm.x
            marker.pose.position.y = lm.y
            marker.pose.position.z = 0.0

            marker.scale.x = marker.scale.y = marker.scale.z = lm.radius * 2.0  # diameter

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            markers.markers.append(marker)

        self.landmark_pub.publish(markers)

    def odom_callback(self, msg):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Robot position
        marker.pose = msg.pose.pose

        # Size of the arrow
        marker.scale.x = 0.3  # length
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Blue arrow
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.odom_pub.publish(marker)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    viz = LandmarkVisualizer()
    viz.run()

