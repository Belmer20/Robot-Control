;; Auto-generated. Do not edit!


(when (boundp 'jetbot_mini::State)
  (if (not (find-package "JETBOT_MINI"))
    (make-package "JETBOT_MINI"))
  (shadow 'State (find-package "JETBOT_MINI")))
(unless (find-package "JETBOT_MINI::STATE")
  (make-package "JETBOT_MINI::STATE"))

(in-package "ROS")
;;//! \htmlinclude State.msg.html


(defclass jetbot_mini::State
  :super ros::object
  :slots (_ready ))

(defmethod jetbot_mini::State
  (:init
   (&key
    ((:ready __ready) nil)
    )
   (send-super :init)
   (setq _ready __ready)
   self)
  (:ready
   (&optional __ready)
   (if __ready (setq _ready __ready)) _ready)
  (:serialization-length
   ()
   (+
    ;; bool _ready
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ready
       (if _ready (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ready
     (setq _ready (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get jetbot_mini::State :md5sum-) "6f378c6311f9e6ccd2cd8c5b327003f1")
(setf (get jetbot_mini::State :datatype-) "jetbot_mini/State")
(setf (get jetbot_mini::State :definition-)
      "bool ready
")



(provide :jetbot_mini/State "6f378c6311f9e6ccd2cd8c5b327003f1")


