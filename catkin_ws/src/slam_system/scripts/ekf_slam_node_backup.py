#!/usr/bin/env python3

import rospy
import numpy as np
import tf

from math import sin, cos, atan2, sqrt
from nav_msgs.msg import Odometry
from my_slam_interfaces.msg import Landmark, LandmarkArray
from visualization_msgs.msg import Marker, MarkerArray


def normalize_angle(angle):
    """Normalize an angle to the range [-pi, pi]."""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle


class EKFSLAM:
    def __init__(self):
        rospy.init_node('ekf_slam_node')

        # --- State and Covariance ---
        self.mu = np.zeros((3, 1))
        self.Sigma = np.diag([0.01, 0.01, np.deg2rad(1.0)])

        # --- Landmark Management ---
        self.landmark_map = {}
        self.next_landmark_index = 0

        # --- Noise Parameters ---
        self.motion_noise_covariance = np.diag([0.1**2, np.deg2rad(5.0)**2])  # R matrix (v, omega)
        self.sensor_noise_covariance = np.diag([0.1**2, np.deg2rad(5)**2])    # Q matrix (range, bearing)

        # --- ROS Communication ---
        self.last_odom_time = None
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/landmarks', LandmarkArray, self.landmark_callback)
        self.map_pub = rospy.Publisher('/slam/map_markers', MarkerArray, queue_size=10)

        # --- TF ---
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        rospy.Timer(rospy.Duration(0.05), self.publish_tf)  # 20Hz
        rospy.loginfo("EKF SLAM Node Started.")

    def odom_callback(self, msg):
        # --- EKF Prediction Step ---
        if self.last_odom_time is None:
            self.last_odom_time = msg.header.stamp
            return

        dt = (msg.header.stamp - self.last_odom_time).to_sec()
        self.last_odom_time = msg.header.stamp
        if dt <= 0:
            return

        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        theta = self.mu[2, 0]

        N = self.mu.shape[0]
        Fx = np.hstack((np.eye(3), np.zeros((3, N - 3))))

        if abs(omega) < 1e-6:  # Linear motion
            motion_update = np.array([
                [v * dt * cos(theta)],
                [v * dt * sin(theta)],
                [0]
            ])
            self.mu = self.mu + Fx.T @ motion_update

            G_robot = np.array([
                [1, 0, -v * dt * sin(theta)],
                [0, 1,  v * dt * cos(theta)],
                [0, 0, 1]
            ])
        else:  # Circular motion
            radius = v / omega
            motion_update = np.array([
                [-radius * sin(theta) + radius * sin(theta + omega * dt)],
                [ radius * cos(theta) - radius * cos(theta + omega * dt)],
                [omega * dt]
            ])
            self.mu = self.mu + Fx.T @ motion_update

            g13 = -radius * cos(theta) + radius * cos(theta + omega * dt)
            g23 = -radius * sin(theta) + radius * sin(theta + omega * dt)
            G_robot = np.array([
                [1, 0, g13],
                [0, 1, g23],
                [0, 0, 1]
            ])

        self.mu[2, 0] = normalize_angle(self.mu[2, 0])

        G = np.eye(N)
        G[:3, :3] = G_robot

        # Jacobian for motion noise mapping
        V = np.array([
            [dt * cos(theta), -0.5 * v * dt**2 * sin(theta)],
            [dt * sin(theta),  0.5 * v * dt**2 * cos(theta)],
            [0, dt]
        ])
        R = V @ self.motion_noise_covariance @ V.T
        self.Sigma = G @ self.Sigma @ G.T + Fx.T @ R @ Fx

    def landmark_callback(self, msg):
        for landmark_obs in msg.landmarks:
            landmark_id = landmark_obs.id
            lx, ly = landmark_obs.x, landmark_obs.y

            # Convert landmark from cartesian (x,y) to polar (range, bearing)
            z = np.array([[sqrt(lx**2 + ly**2)], [atan2(ly, lx)]])

            if landmark_id not in self.landmark_map:
                self.initialize_landmark(landmark_id, z)
            else:
                self.update_landmark(landmark_id, z)

        self.publish_map_markers()

    def initialize_landmark(self, landmark_id, z):
        rospy.loginfo(f"Initializing new landmark with ID: {landmark_id}")
        self.landmark_map[landmark_id] = self.next_landmark_index
        self.next_landmark_index += 1

        robot_x, robot_y, robot_theta = self.mu[0, 0], self.mu[1, 0], self.mu[2, 0]
        r, phi = z[0, 0], z[1, 0]

        # Inverse measurement model
        landmark_x = robot_x + r * cos(phi + robot_theta)
        landmark_y = robot_y + r * sin(phi + robot_theta)
        new_landmark_mu = np.array([[landmark_x], [landmark_y]])

        # Augment state vector and covariance
        self.mu = np.vstack((self.mu, new_landmark_mu))

        old_size = self.Sigma.shape[0]
        new_Sigma = np.eye(old_size + 2) * 1e-4
        new_Sigma[:old_size, :old_size] = self.Sigma
        new_Sigma[old_size:, old_size:] = np.eye(2) * 1000
        self.Sigma = new_Sigma

    def update_landmark(self, landmark_id, z):
        N = self.mu.shape[0]
        idx = self.landmark_map[landmark_id] * 2 + 3

        robot_x, robot_y, robot_theta = self.mu[0, 0], self.mu[1, 0], self.mu[2, 0]
        landmark_x, landmark_y = self.mu[idx, 0], self.mu[idx + 1, 0]

        # Expected observation
        delta_x = landmark_x - robot_x
        delta_y = landmark_y - robot_y
        q = delta_x**2 + delta_y**2
        z_hat = np.array([
            [sqrt(q)],
            [normalize_angle(atan2(delta_y, delta_x) - robot_theta)]
        ])

        # Measurement jacobian
        H_low = np.array([
            [-delta_x / sqrt(q), -delta_y / sqrt(q), 0, delta_x / sqrt(q), delta_y / sqrt(q)],
            [ delta_y / q,       -delta_x / q,     -1, -delta_y / q,      delta_x / q]
        ])

        H = np.zeros((2, N))
        H[:, :3] = H_low[:, :3]
        H[:, idx:idx + 2] = H_low[:, 3:5]

        # EKF Update
        S = H @ self.Sigma @ H.T + self.sensor_noise_covariance
        K = self.Sigma @ H.T @ np.linalg.inv(S)
        innovation = z - z_hat
        innovation[1] = normalize_angle(innovation[1])

        self.mu = self.mu + K @ innovation
        self.Sigma = (np.eye(N) - K @ H) @ self.Sigma
        self.mu[2, 0] = normalize_angle(self.mu[2, 0])

    def publish_tf(self, event):
        # Publish T_map_odom = T_map_robot * inv(T_odom_robot)

        # EKF pose
        ekf_pose = self.mu[:3].flatten()
        T_map_robot_mat = tf.transformations.compose_matrix(
            translate=(ekf_pose[0], ekf_pose[1], 0),
            angles=(0, 0, ekf_pose[2])
        )

        try:
            (odom_trans, odom_rot) = self.tf_listener.lookupTransform(
                '/odom', '/base_link', rospy.Time(0)
            )
            T_odom_robot_mat = tf.transformations.compose_matrix(
                translate=odom_trans,
                angles=tf.transformations.euler_from_quaternion(odom_rot)
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(1.0, "Could not get /odom -> /base_link transform. Skipping TF broadcast.")
            return

        T_map_odom_mat = np.dot(T_map_robot_mat, np.linalg.inv(T_odom_robot_mat))
        map_odom_trans = tf.transformations.translation_from_matrix(T_map_odom_mat)
        map_odom_quat = tf.transformations.quaternion_from_matrix(T_map_odom_mat)

        self.tf_broadcaster.sendTransform(
            map_odom_trans,
            map_odom_quat,
            rospy.Time.now(),
            "odom",  # Child frame
            "map"    # Parent frame
        )

    def publish_map_markers(self):
        marker_array = MarkerArray()
        for landmark_id, index in self.landmark_map.items():
            idx_in_mu = index * 2 + 3
            if idx_in_mu >= len(self.mu):
                continue

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "slam_map"
            marker.id = landmark_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = self.mu[idx_in_mu, 0]
            marker.pose.position.y = self.mu[idx_in_mu + 1, 0]
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.2

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.8

            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)

        self.map_pub.publish(marker_array)


if __name__ == '__main__':
    try:
        slam = EKFSLAM()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

