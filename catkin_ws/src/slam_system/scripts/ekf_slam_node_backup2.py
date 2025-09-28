#!/usr/bin/env python3

import rospy
import numpy as np
import tf
from math import sin, cos, atan2, sqrt

from nav_msgs.msg import Odometry
from my_slam_interfaces.msg import LandmarkArray
from visualization_msgs.msg import Marker, MarkerArray

def normalize_angle(angle):
    """ Normalize an angle to the range [-pi, pi] """
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
        self.next_landmark_id = 0

        # --- Noise Parameters ---
        self.motion_noise_R = np.diag([
            0.1**2,            # Uncertainty in x
            0.1**2,            # Uncertainty in y
            np.deg2rad(1.0)**2 # Uncertainty in theta
        ])
        self.sensor_noise_covariance = np.diag([0.1**2, np.deg2rad(5)**2])   # Q matrix (range, bearing)

        # --- Data Association Gate ---
        self.GATE_THRESHOLD = 5.99

        # --- ROS Communication ---
        self.last_odom_time = None
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/landmarks', LandmarkArray, self.landmark_callback)
        self.map_pub = rospy.Publisher('/slam/map_markers', MarkerArray, queue_size=10)
        
        # --- TF ---
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        rospy.Timer(rospy.Duration(0.05), self.publish_tf) # 20Hz
        rospy.loginfo("EKF SLAM Node with Robust Data Association Started.")

    def odom_callback(self, msg):
        if self.last_odom_time is None:
            self.last_odom_time = msg.header.stamp
            return
        dt = (msg.header.stamp - self.last_odom_time).to_sec()
        if dt <= 0: return
        self.last_odom_time = msg.header.stamp
        
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        theta = self.mu[2, 0]
        
        self.mu[0, 0] += v * cos(theta) * dt
        self.mu[1, 0] += v * sin(theta) * dt
        self.mu[2, 0] = normalize_angle(self.mu[2, 0] + omega * dt)
        
        N = self.mu.shape[0]
        G_robot = np.array([
            [1, 0, -v * sin(theta) * dt],
            [0, 1,  v * cos(theta) * dt],
            [0, 0, 1]
        ])
        G = np.eye(N)
        G[:3, :3] = G_robot
        R_full = np.zeros((N, N))
        R_full[:3, :3] = self.motion_noise_R
        self.Sigma = G @ self.Sigma @ G.T + R_full

    def landmark_callback(self, msg):
        for landmark_obs in msg.landmarks:
            lx = landmark_obs.x
            ly = landmark_obs.y
            z = np.array([[sqrt(lx**2 + ly**2)], [atan2(ly, lx)]])

            best_match_idx, min_dist = self.data_association(z)
            
            if best_match_idx != -1 and min_dist < self.GATE_THRESHOLD:
                rospy.logdebug(f"Associated observation with landmark index {best_match_idx} (dist={min_dist:.2f})")
                self.update_landmark(best_match_idx, z)
            else:
                rospy.loginfo(f"Initializing new landmark (min_dist={min_dist:.2f})")
                self.initialize_landmark(z)
        
        self.publish_map_markers()

    def data_association(self, z):
        min_dist = float('inf')
        best_match_idx = -1
        
        num_landmarks = (len(self.mu) - 3) // 2
        for i in range(num_landmarks):
            z_hat, H, S = self.predict_observation_model(i)
            innovation = z - z_hat
            innovation[1] = normalize_angle(innovation[1])
            
            # --- FIX: Use .item() to extract the scalar value from the 1x1 numpy array ---
            dist = (innovation.T @ np.linalg.inv(S) @ innovation).item()
            
            if dist < min_dist:
                min_dist = dist
                best_match_idx = i
                
        return best_match_idx, min_dist

    def predict_observation_model(self, landmark_index):
        N = self.mu.shape[0]
        idx_in_mu = 3 + landmark_index * 2
        robot_x, robot_y, robot_theta = self.mu[0,0], self.mu[1,0], self.mu[2,0]
        landmark_x, landmark_y = self.mu[idx_in_mu,0], self.mu[idx_in_mu+1,0]
        
        delta_x = landmark_x - robot_x
        delta_y = landmark_y - robot_y
        q = delta_x**2 + delta_y**2
        
        z_hat = np.array([[sqrt(q)], [normalize_angle(atan2(delta_y, delta_x) - robot_theta)]])
        
        H_low = np.array([
            [-delta_x/sqrt(q), -delta_y/sqrt(q), 0, delta_x/sqrt(q), delta_y/sqrt(q)],
            [ delta_y/q,       -delta_x/q,     -1, -delta_y/q,      delta_x/q]
        ])
        
        H = np.zeros((2, N))
        H[:, :3] = H_low[:, :3]
        H[:, idx_in_mu:idx_in_mu+2] = H_low[:, 3:5]
        
        S = H @ self.Sigma @ H.T + self.sensor_noise_covariance
        
        return z_hat, H, S

    def initialize_landmark(self, z):
        new_id = self.next_landmark_id
        self.landmark_map[new_id] = (len(self.mu) - 3) // 2
        self.next_landmark_id += 1

        robot_x, robot_y, robot_theta = self.mu[0,0], self.mu[1,0], self.mu[2,0]
        r, phi = z[0,0], z[1,0]
        
        landmark_x = robot_x + r * cos(phi + robot_theta)
        landmark_y = robot_y + r * sin(phi + robot_theta)
        new_landmark_mu = np.array([[landmark_x], [landmark_y]])
        
        self.mu = np.vstack((self.mu, new_landmark_mu))
        
        old_size = self.Sigma.shape[0]
        new_Sigma = np.eye(old_size + 2) * 1e-4
        new_Sigma[:old_size, :old_size] = self.Sigma
        new_Sigma[old_size:, old_size:] = np.eye(2) * 1000
        self.Sigma = new_Sigma

    def update_landmark(self, landmark_index, z):
        z_hat, H, S = self.predict_observation_model(landmark_index)
        
        K = self.Sigma @ H.T @ np.linalg.inv(S)
        
        innovation = z - z_hat
        innovation[1] = normalize_angle(innovation[1])
        
        self.mu = self.mu + K @ innovation
        self.Sigma = (np.eye(self.mu.shape[0]) - K @ H) @ self.Sigma
        self.mu[2,0] = normalize_angle(self.mu[2,0])

    def publish_tf(self, event):
        ekf_pose = self.mu[:3].flatten()
        T_map_robot_mat = tf.transformations.compose_matrix(
            translate=(ekf_pose[0], ekf_pose[1], 0),
            angles=(0, 0, ekf_pose[2])
        )
        try:
            (odom_trans, odom_rot) = self.tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            T_odom_robot_mat = tf.transformations.compose_matrix(
                translate=odom_trans,
                angles=tf.transformations.euler_from_quaternion(odom_rot)
            )
            T_map_odom_mat = np.dot(T_map_robot_mat, np.linalg.inv(T_odom_robot_mat))
            map_odom_trans = tf.transformations.translation_from_matrix(T_map_odom_mat)
            map_odom_quat = tf.transformations.quaternion_from_matrix(T_map_odom_mat)
            self.tf_broadcaster.sendTransform(map_odom_trans, map_odom_quat, rospy.Time.now(), "odom", "map")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(1.0, "Could not get /odom -> /base_link transform. Skipping TF broadcast.")

    def publish_map_markers(self):
        marker_array = MarkerArray()
        for landmark_id, index in self.landmark_map.items():
            idx_in_mu = 3 + index * 2
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "slam_map"
            marker.id = landmark_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = self.mu[idx_in_mu, 0]
            marker.pose.position.y = self.mu[idx_in_mu+1, 0]
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.0, 1.0, 1.0, 0.8)
            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)
        self.map_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        slam = EKFSLAM()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

