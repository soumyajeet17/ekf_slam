;; Auto-generated. Do not edit!


(when (boundp 'my_slam_interfaces::LandmarkArray)
  (if (not (find-package "MY_SLAM_INTERFACES"))
    (make-package "MY_SLAM_INTERFACES"))
  (shadow 'LandmarkArray (find-package "MY_SLAM_INTERFACES")))
(unless (find-package "MY_SLAM_INTERFACES::LANDMARKARRAY")
  (make-package "MY_SLAM_INTERFACES::LANDMARKARRAY"))

(in-package "ROS")
;;//! \htmlinclude LandmarkArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass my_slam_interfaces::LandmarkArray
  :super ros::object
  :slots (_header _landmarks ))

(defmethod my_slam_interfaces::LandmarkArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:landmarks __landmarks) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _landmarks __landmarks)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:landmarks
   (&rest __landmarks)
   (if (keywordp (car __landmarks))
       (send* _landmarks __landmarks)
     (progn
       (if __landmarks (setq _landmarks (car __landmarks)))
       _landmarks)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; my_slam_interfaces/Landmark[] _landmarks
    (apply #'+ (send-all _landmarks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; my_slam_interfaces/Landmark[] _landmarks
     (write-long (length _landmarks) s)
     (dolist (elem _landmarks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; my_slam_interfaces/Landmark[] _landmarks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _landmarks (let (r) (dotimes (i n) (push (instance my_slam_interfaces::Landmark :init) r)) r))
     (dolist (elem- _landmarks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get my_slam_interfaces::LandmarkArray :md5sum-) "f50d5bcb520887abb25781659852ffdf")
(setf (get my_slam_interfaces::LandmarkArray :datatype-) "my_slam_interfaces/LandmarkArray")
(setf (get my_slam_interfaces::LandmarkArray :definition-)
      "std_msgs/Header header
Landmark[] landmarks


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: my_slam_interfaces/Landmark
float32 x
float32 y
float32 radius
int32 id

")



(provide :my_slam_interfaces/LandmarkArray "f50d5bcb520887abb25781659852ffdf")


