;; Auto-generated. Do not edit!


(when (boundp 'jetbot_mini::Command_classes)
  (if (not (find-package "JETBOT_MINI"))
    (make-package "JETBOT_MINI"))
  (shadow 'Command_classes (find-package "JETBOT_MINI")))
(unless (find-package "JETBOT_MINI::COMMAND_CLASSES")
  (make-package "JETBOT_MINI::COMMAND_CLASSES"))

(in-package "ROS")
;;//! \htmlinclude Command_classes.msg.html


(defclass jetbot_mini::Command_classes
  :super ros::object
  :slots (_action _subject _value _unit ))

(defmethod jetbot_mini::Command_classes
  (:init
   (&key
    ((:action __action) "")
    ((:subject __subject) "")
    ((:value __value) "")
    ((:unit __unit) "")
    )
   (send-super :init)
   (setq _action (string __action))
   (setq _subject (string __subject))
   (setq _value (string __value))
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
    ;; string _value
    4 (length _value)
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
     ;; string _value
       (write-long (length _value) s) (princ _value s)
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
   ;; string _value
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _value (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _unit
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _unit (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get jetbot_mini::Command_classes :md5sum-) "06f8489ff1e7fcaba60778ad9099d7c2")
(setf (get jetbot_mini::Command_classes :datatype-) "jetbot_mini/Command_classes")
(setf (get jetbot_mini::Command_classes :definition-)
      "string action
string subject
string value
string unit
")



(provide :jetbot_mini/Command_classes "06f8489ff1e7fcaba60778ad9099d7c2")


