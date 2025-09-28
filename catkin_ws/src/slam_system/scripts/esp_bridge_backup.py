#!/usr/bin/env python3

import rospy
import tf
from math import sin, cos, pi

from my_slam_interfaces.msg import Ticks, LandmarkArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from visualization_msgs.msg import Marker, MarkerArray


class RobotBridge:
    def __init__(self):
        rospy.init_node('esp32_bridge_node', anonymous=True)

        # --- Get Robot Parameters ---
        self.wheel_base = rospy.get_param('~wheel_base', 0.155)
        self.ticks_per_meter = rospy.get_param('~ticks_per_meter', 191)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')

        # --- Internal State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0       # Velocity in x
        self.vth = 0.0      # Angular velocity in z
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_time = rospy.Time.now()

        # --- ROS Subscribers ---
        rospy.Subscriber("robot_ticks", Ticks, self.ticks_callback)
        rospy.Subscriber("landmarks", LandmarkArray, self.landmarks_callback)

        # --- ROS Publishers ---
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.marker_pub = rospy.Publisher("landmark_markers", MarkerArray, queue_size=10)

        # --- TF Broadcaster ---
        self.odom_broadcaster = tf.TransformBroadcaster()

        rospy.loginfo("ESP32 Bridge Node Started.")

    def ticks_callback(self, msg):
        current_time = msg.header.stamp

        # Initialize last ticks on first message
        if self.last_left_ticks is None:
            self.last_left_ticks = msg.left_ticks
            self.last_right_ticks = msg.right_ticks
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).to_sec()
        if dt <= 0:  # Avoid division by zero
            return

        # Calculate delta ticks
        delta_left = msg.left_ticks - self.last_left_ticks
        delta_right = msg.right_ticks - self.last_right_ticks

        # Convert ticks to meters
        dist_left = delta_left / self.ticks_per_meter
        dist_right = delta_right / self.ticks_per_meter

        # Calculate robot's change in position
        delta_s = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_base

        # *** CORRECTED PART: Calculate velocities ***
        self.vx = delta_s / dt        # Linear velocity
        self.vth = delta_theta / dt   # Angular velocity

        # Update pose (midpoint method for better accuracy)
        self.x += delta_s * cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # --- Publish Odometry and TF ---
        self.publish_odometry(current_time)

        # Update last values
        self.last_left_ticks = msg.left_ticks
        self.last_right_ticks = msg.right_ticks
        self.last_time = current_time

    def publish_odometry(self, current_time):
        # Create quaternion from yaw (theta)
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        # Broadcast the transform from odom -> base_link
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            current_time,
            self.base_frame,
            self.odom_frame
        )

        # Publish the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # *** CORRECTED PART: Publish calculated velocities ***
        odom.twist.twist = Twist(Vector3(self.vx, 0, 0), Vector3(0, 0, self.vth))

        self.odom_pub.publish(odom)

    def landmarks_callback(self, msg):
        marker_array = MarkerArray()

        # Colors for different landmark IDs
        colors = [
            (1, 0, 0, 1),  # Red
            (0, 1, 0, 1),  # Green
            (0, 0, 1, 1),  # Blue
            (1, 1, 0, 1),  # Yellow
            (0, 1, 1, 1)   # Cyan
        ]

        for landmark in msg.landmarks:
            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "landmarks"
            marker.id = landmark.id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = landmark.x
            marker.pose.position.y = landmark.y
            marker.pose.position.z = 0.1  # Center of cylinder at 10cm height
            marker.pose.orientation.w = 1.0

            marker.scale.x = landmark.radius * 2  # Diameter
            marker.scale.y = landmark.radius * 2  # Diameter
            marker.scale.z = 0.2  # 20cm height

            color_index = landmark.id % len(colors)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = colors[color_index]

            marker.lifetime = rospy.Duration(1.0)  # Marker lasts for 1 second
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


if __name__ == '__main__':
    try:
        bridge = RobotBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

