;; Auto-generated. Do not edit!


(when (boundp 'my_slam_interfaces::Ticks)
  (if (not (find-package "MY_SLAM_INTERFACES"))
    (make-package "MY_SLAM_INTERFACES"))
  (shadow 'Ticks (find-package "MY_SLAM_INTERFACES")))
(unless (find-package "MY_SLAM_INTERFACES::TICKS")
  (make-package "MY_SLAM_INTERFACES::TICKS"))

(in-package "ROS")
;;//! \htmlinclude Ticks.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass my_slam_interfaces::Ticks
  :super ros::object
  :slots (_header _left_ticks _right_ticks ))

(defmethod my_slam_interfaces::Ticks
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left_ticks __left_ticks) 0)
    ((:right_ticks __right_ticks) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _left_ticks (round __left_ticks))
   (setq _right_ticks (round __right_ticks))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left_ticks
   (&optional __left_ticks)
   (if __left_ticks (setq _left_ticks __left_ticks)) _left_ticks)
  (:right_ticks
   (&optional __right_ticks)
   (if __right_ticks (setq _right_ticks __right_ticks)) _right_ticks)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int64 _left_ticks
    8
    ;; int64 _right_ticks
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int64 _left_ticks
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _left_ticks (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _left_ticks) (= (length (_left_ticks . bv)) 2)) ;; bignum
              (write-long (ash (elt (_left_ticks . bv) 0) 0) s)
              (write-long (ash (elt (_left_ticks . bv) 1) -1) s))
             ((and (class _left_ticks) (= (length (_left_ticks . bv)) 1)) ;; big1
              (write-long (elt (_left_ticks . bv) 0) s)
              (write-long (if (>= _left_ticks 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _left_ticks s)(write-long (if (>= _left_ticks 0) 0 #xffffffff) s)))
     ;; int64 _right_ticks
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _right_ticks (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _right_ticks) (= (length (_right_ticks . bv)) 2)) ;; bignum
              (write-long (ash (elt (_right_ticks . bv) 0) 0) s)
              (write-long (ash (elt (_right_ticks . bv) 1) -1) s))
             ((and (class _right_ticks) (= (length (_right_ticks . bv)) 1)) ;; big1
              (write-long (elt (_right_ticks . bv) 0) s)
              (write-long (if (>= _right_ticks 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _right_ticks s)(write-long (if (>= _right_ticks 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int64 _left_ticks
#+(or :alpha :irix6 :x86_64)
      (setf _left_ticks (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _left_ticks (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _right_ticks
#+(or :alpha :irix6 :x86_64)
      (setf _right_ticks (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _right_ticks (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get my_slam_interfaces::Ticks :md5sum-) "b2e3ba80aef7601eec29b4f6edd523f2")
(setf (get my_slam_interfaces::Ticks :datatype-) "my_slam_interfaces/Ticks")
(setf (get my_slam_interfaces::Ticks :definition-)
      "std_msgs/Header header
int64 left_ticks
int64 right_ticks


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

")



(provide :my_slam_interfaces/Ticks "b2e3ba80aef7601eec29b4f6edd523f2")


