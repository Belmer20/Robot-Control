;; Auto-generated. Do not edit!


(when (boundp 'jetbot_mini::State_msg)
  (if (not (find-package "JETBOT_MINI"))
    (make-package "JETBOT_MINI"))
  (shadow 'State_msg (find-package "JETBOT_MINI")))
(unless (find-package "JETBOT_MINI::STATE_MSG")
  (make-package "JETBOT_MINI::STATE_MSG"))

(in-package "ROS")
;;//! \htmlinclude State_msg.msg.html


(defclass jetbot_mini::State_msg
  :super ros::object
  :slots (_index _ready ))

(defmethod jetbot_mini::State_msg
  (:init
   (&key
    ((:index __index) 0)
    ((:ready __ready) nil)
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _ready __ready)
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:ready
   (&optional __ready)
   (if __ready (setq _ready __ready)) _ready)
  (:serialization-length
   ()
   (+
    ;; int32 _index
    4
    ;; bool _ready
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _index
       (write-long _index s)
     ;; bool _ready
       (if _ready (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _index
     (setq _index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _ready
     (setq _ready (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get jetbot_mini::State_msg :md5sum-) "9d2833016bdbb0ca11d012b3bd7a19bf")
(setf (get jetbot_mini::State_msg :datatype-) "jetbot_mini/State_msg")
(setf (get jetbot_mini::State_msg :definition-)
      "int32 index
bool ready
")



(provide :jetbot_mini/State_msg "9d2833016bdbb0ca11d012b3bd7a19bf")

