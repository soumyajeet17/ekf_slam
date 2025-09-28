; Auto-generated. Do not edit!


(cl:in-package my_slam_interfaces-msg)


;//! \htmlinclude Ticks.msg.html

(cl:defclass <Ticks> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (left_ticks
    :reader left_ticks
    :initarg :left_ticks
    :type cl:integer
    :initform 0)
   (right_ticks
    :reader right_ticks
    :initarg :right_ticks
    :type cl:integer
    :initform 0))
)

(cl:defclass Ticks (<Ticks>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Ticks>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Ticks)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_slam_interfaces-msg:<Ticks> is deprecated: use my_slam_interfaces-msg:Ticks instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Ticks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_slam_interfaces-msg:header-val is deprecated.  Use my_slam_interfaces-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'left_ticks-val :lambda-list '(m))
(cl:defmethod left_ticks-val ((m <Ticks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_slam_interfaces-msg:left_ticks-val is deprecated.  Use my_slam_interfaces-msg:left_ticks instead.")
  (left_ticks m))

(cl:ensure-generic-function 'right_ticks-val :lambda-list '(m))
(cl:defmethod right_ticks-val ((m <Ticks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_slam_interfaces-msg:right_ticks-val is deprecated.  Use my_slam_interfaces-msg:right_ticks instead.")
  (right_ticks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Ticks>) ostream)
  "Serializes a message object of type '<Ticks>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'left_ticks)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right_ticks)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Ticks>) istream)
  "Deserializes a message object of type '<Ticks>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_ticks) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_ticks) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Ticks>)))
  "Returns string type for a message object of type '<Ticks>"
  "my_slam_interfaces/Ticks")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ticks)))
  "Returns string type for a message object of type 'Ticks"
  "my_slam_interfaces/Ticks")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Ticks>)))
  "Returns md5sum for a message object of type '<Ticks>"
  "b2e3ba80aef7601eec29b4f6edd523f2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Ticks)))
  "Returns md5sum for a message object of type 'Ticks"
  "b2e3ba80aef7601eec29b4f6edd523f2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Ticks>)))
  "Returns full string definition for message of type '<Ticks>"
  (cl:format cl:nil "std_msgs/Header header~%int64 left_ticks~%int64 right_ticks~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Ticks)))
  "Returns full string definition for message of type 'Ticks"
  (cl:format cl:nil "std_msgs/Header header~%int64 left_ticks~%int64 right_ticks~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Ticks>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Ticks>))
  "Converts a ROS message object to a list"
  (cl:list 'Ticks
    (cl:cons ':header (header msg))
    (cl:cons ':left_ticks (left_ticks msg))
    (cl:cons ':right_ticks (right_ticks msg))
))
