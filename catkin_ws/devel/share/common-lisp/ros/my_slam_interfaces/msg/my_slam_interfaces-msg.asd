
(cl:in-package :asdf)

(defsystem "my_slam_interfaces-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Landmark" :depends-on ("_package_Landmark"))
    (:file "_package_Landmark" :depends-on ("_package"))
    (:file "LandmarkArray" :depends-on ("_package_LandmarkArray"))
    (:file "_package_LandmarkArray" :depends-on ("_package"))
    (:file "Ticks" :depends-on ("_package_Ticks"))
    (:file "_package_Ticks" :depends-on ("_package"))
  ))