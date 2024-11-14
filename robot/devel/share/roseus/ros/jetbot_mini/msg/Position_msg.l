;; Auto-generated. Do not edit!


(when (boundp 'jetbot_mini::Position_msg)
  (if (not (find-package "JETBOT_MINI"))
    (make-package "JETBOT_MINI"))
  (shadow 'Position_msg (find-package "JETBOT_MINI")))
(unless (find-package "JETBOT_MINI::POSITION_MSG")
  (make-package "JETBOT_MINI::POSITION_MSG"))

(in-package "ROS")
;;//! \htmlinclude Position_msg.msg.html


(defclass jetbot_mini::Position_msg
  :super ros::object
  :slots (_robot_index _x _y ))

(defmethod jetbot_mini::Position_msg
  (:init
   (&key
    ((:robot_index __robot_index) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _robot_index (round __robot_index))
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:robot_index
   (&optional __robot_index)
   (if __robot_index (setq _robot_index __robot_index)) _robot_index)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; int32 _robot_index
    4
    ;; float64 _x
    8
    ;; float64 _y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _robot_index
       (write-long _robot_index s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _robot_index
     (setq _robot_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get jetbot_mini::Position_msg :md5sum-) "c8685de48e63ca6aa63f0d3bc9a2af72")
(setf (get jetbot_mini::Position_msg :datatype-) "jetbot_mini/Position_msg")
(setf (get jetbot_mini::Position_msg :definition-)
      "int32 robot_index
float64 x
float64 y


")



(provide :jetbot_mini/Position_msg "c8685de48e63ca6aa63f0d3bc9a2af72")


