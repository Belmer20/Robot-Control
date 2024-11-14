
(cl:in-package :asdf)

(defsystem "secondary_robot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Position_msg" :depends-on ("_package_Position_msg"))
    (:file "_package_Position_msg" :depends-on ("_package"))
    (:file "State_msg" :depends-on ("_package_State_msg"))
    (:file "_package_State_msg" :depends-on ("_package"))
  ))