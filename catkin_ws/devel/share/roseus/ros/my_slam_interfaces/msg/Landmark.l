;; Auto-generated. Do not edit!


(when (boundp 'my_slam_interfaces::Landmark)
  (if (not (find-package "MY_SLAM_INTERFACES"))
    (make-package "MY_SLAM_INTERFACES"))
  (shadow 'Landmark (find-package "MY_SLAM_INTERFACES")))
(unless (find-package "MY_SLAM_INTERFACES::LANDMARK")
  (make-package "MY_SLAM_INTERFACES::LANDMARK"))

(in-package "ROS")
;;//! \htmlinclude Landmark.msg.html


(defclass my_slam_interfaces::Landmark
  :super ros::object
  :slots (_x _y _radius _id ))

(defmethod my_slam_interfaces::Landmark
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:radius __radius) 0.0)
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _radius (float __radius))
   (setq _id (round __id))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _radius
    4
    ;; int32 _id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _id
       (write-long _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _radius
     (setq _radius (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get my_slam_interfaces::Landmark :md5sum-) "82af38ba43a2294416e5d394d28eaa70")
(setf (get my_slam_interfaces::Landmark :datatype-) "my_slam_interfaces/Landmark")
(setf (get my_slam_interfaces::Landmark :definition-)
      "float32 x
float32 y
float32 radius
int32 id

")



(provide :my_slam_interfaces/Landmark "82af38ba43a2294416e5d394d28eaa70")


