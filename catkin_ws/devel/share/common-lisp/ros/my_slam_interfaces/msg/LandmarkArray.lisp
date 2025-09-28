; Auto-generated. Do not edit!


(cl:in-package my_slam_interfaces-msg)


;//! \htmlinclude LandmarkArray.msg.html

(cl:defclass <LandmarkArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (landmarks
    :reader landmarks
    :initarg :landmarks
    :type (cl:vector my_slam_interfaces-msg:Landmark)
   :initform (cl:make-array 0 :element-type 'my_slam_interfaces-msg:Landmark :initial-element (cl:make-instance 'my_slam_interfaces-msg:Landmark))))
)

(cl:defclass LandmarkArray (<LandmarkArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LandmarkArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LandmarkArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_slam_interfaces-msg:<LandmarkArray> is deprecated: use my_slam_interfaces-msg:LandmarkArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LandmarkArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_slam_interfaces-msg:header-val is deprecated.  Use my_slam_interfaces-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'landmarks-val :lambda-list '(m))
(cl:defmethod landmarks-val ((m <LandmarkArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_slam_interfaces-msg:landmarks-val is deprecated.  Use my_slam_interfaces-msg:landmarks instead.")
  (landmarks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LandmarkArray>) ostream)
  "Serializes a message object of type '<LandmarkArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'landmarks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'landmarks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LandmarkArray>) istream)
  "Deserializes a message object of type '<LandmarkArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'landmarks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'landmarks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'my_slam_interfaces-msg:Landmark))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LandmarkArray>)))
  "Returns string type for a message object of type '<LandmarkArray>"
  "my_slam_interfaces/LandmarkArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LandmarkArray)))
  "Returns string type for a message object of type 'LandmarkArray"
  "my_slam_interfaces/LandmarkArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LandmarkArray>)))
  "Returns md5sum for a message object of type '<LandmarkArray>"
  "f50d5bcb520887abb25781659852ffdf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LandmarkArray)))
  "Returns md5sum for a message object of type 'LandmarkArray"
  "f50d5bcb520887abb25781659852ffdf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LandmarkArray>)))
  "Returns full string definition for message of type '<LandmarkArray>"
  (cl:format cl:nil "std_msgs/Header header~%Landmark[] landmarks~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: my_slam_interfaces/Landmark~%float32 x~%float32 y~%float32 radius~%int32 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LandmarkArray)))
  "Returns full string definition for message of type 'LandmarkArray"
  (cl:format cl:nil "std_msgs/Header header~%Landmark[] landmarks~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: my_slam_interfaces/Landmark~%float32 x~%float32 y~%float32 radius~%int32 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LandmarkArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'landmarks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LandmarkArray>))
  "Converts a ROS message object to a list"
  (cl:list 'LandmarkArray
    (cl:cons ':header (header msg))
    (cl:cons ':landmarks (landmarks msg))
))
