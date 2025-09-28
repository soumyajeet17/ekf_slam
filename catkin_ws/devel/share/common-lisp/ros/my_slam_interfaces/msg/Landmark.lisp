; Auto-generated. Do not edit!


(cl:in-package my_slam_interfaces-msg)


;//! \htmlinclude Landmark.msg.html

(cl:defclass <Landmark> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0)
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass Landmark (<Landmark>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Landmark>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Landmark)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_slam_interfaces-msg:<Landmark> is deprecated: use my_slam_interfaces-msg:Landmark instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Landmark>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_slam_interfaces-msg:x-val is deprecated.  Use my_slam_interfaces-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Landmark>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_slam_interfaces-msg:y-val is deprecated.  Use my_slam_interfaces-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <Landmark>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_slam_interfaces-msg:radius-val is deprecated.  Use my_slam_interfaces-msg:radius instead.")
  (radius m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Landmark>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_slam_interfaces-msg:id-val is deprecated.  Use my_slam_interfaces-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Landmark>) ostream)
  "Serializes a message object of type '<Landmark>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Landmark>) istream)
  "Deserializes a message object of type '<Landmark>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Landmark>)))
  "Returns string type for a message object of type '<Landmark>"
  "my_slam_interfaces/Landmark")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Landmark)))
  "Returns string type for a message object of type 'Landmark"
  "my_slam_interfaces/Landmark")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Landmark>)))
  "Returns md5sum for a message object of type '<Landmark>"
  "82af38ba43a2294416e5d394d28eaa70")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Landmark)))
  "Returns md5sum for a message object of type 'Landmark"
  "82af38ba43a2294416e5d394d28eaa70")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Landmark>)))
  "Returns full string definition for message of type '<Landmark>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 radius~%int32 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Landmark)))
  "Returns full string definition for message of type 'Landmark"
  (cl:format cl:nil "float32 x~%float32 y~%float32 radius~%int32 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Landmark>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Landmark>))
  "Converts a ROS message object to a list"
  (cl:list 'Landmark
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':radius (radius msg))
    (cl:cons ':id (id msg))
))
