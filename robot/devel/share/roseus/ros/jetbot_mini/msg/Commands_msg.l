;; Auto-generated. Do not edit!


(when (boundp 'jetbot_mini::Commands_msg)
  (if (not (find-package "JETBOT_MINI"))
    (make-package "JETBOT_MINI"))
  (shadow 'Commands_msg (find-package "JETBOT_MINI")))
(unless (find-package "JETBOT_MINI::COMMANDS_MSG")
  (make-package "JETBOT_MINI::COMMANDS_MSG"))

(in-package "ROS")
;;//! \htmlinclude Commands_msg.msg.html


(defclass jetbot_mini::Commands_msg
  :super ros::object
  :slots (_action _subject _value _unit ))

(defmethod jetbot_mini::Commands_msg
  (:init
   (&key
    ((:action __action) "")
    ((:subject __subject) "")
    ((:value __value) 0.0)
    ((:unit __unit) "")
    )
   (send-super :init)
   (setq _action (string __action))
   (setq _subject (string __subject))
   (setq _value (float __value))
   (setq _unit (string __unit))
   self)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:subject
   (&optional __subject)
   (if __subject (setq _subject __subject)) _subject)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:unit
   (&optional __unit)
   (if __unit (setq _unit __unit)) _unit)
  (:serialization-length
   ()
   (+
    ;; string _action
    4 (length _action)
    ;; string _subject
    4 (length _subject)
    ;; float64 _value
    8
    ;; string _unit
    4 (length _unit)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _action
       (write-long (length _action) s) (princ _action s)
     ;; string _subject
       (write-long (length _subject) s) (princ _subject s)
     ;; float64 _value
       (sys::poke _value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _unit
       (write-long (length _unit) s) (princ _unit s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _action
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _action (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _subject
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _subject (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _value
     (setq _value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _unit
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _unit (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get jetbot_mini::Commands_msg :md5sum-) "88c97c65bce824d2b08b3ca7fe1f5632")
(setf (get jetbot_mini::Commands_msg :datatype-) "jetbot_mini/Commands_msg")
(setf (get jetbot_mini::Commands_msg :definition-)
      "string action
string subject
float64 value
string unit
")



(provide :jetbot_mini/Commands_msg "88c97c65bce824d2b08b3ca7fe1f5632")


