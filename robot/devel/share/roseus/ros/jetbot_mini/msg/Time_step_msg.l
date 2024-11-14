;; Auto-generated. Do not edit!


(when (boundp 'jetbot_mini::Time_step_msg)
  (if (not (find-package "JETBOT_MINI"))
    (make-package "JETBOT_MINI"))
  (shadow 'Time_step_msg (find-package "JETBOT_MINI")))
(unless (find-package "JETBOT_MINI::TIME_STEP_MSG")
  (make-package "JETBOT_MINI::TIME_STEP_MSG"))

(in-package "ROS")
;;//! \htmlinclude Time_step_msg.msg.html


(defclass jetbot_mini::Time_step_msg
  :super ros::object
  :slots (_time_step ))

(defmethod jetbot_mini::Time_step_msg
  (:init
   (&key
    ((:time_step __time_step) 0)
    )
   (send-super :init)
   (setq _time_step (round __time_step))
   self)
  (:time_step
   (&optional __time_step)
   (if __time_step (setq _time_step __time_step)) _time_step)
  (:serialization-length
   ()
   (+
    ;; int64 _time_step
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _time_step
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _time_step (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _time_step) (= (length (_time_step . bv)) 2)) ;; bignum
              (write-long (ash (elt (_time_step . bv) 0) 0) s)
              (write-long (ash (elt (_time_step . bv) 1) -1) s))
             ((and (class _time_step) (= (length (_time_step . bv)) 1)) ;; big1
              (write-long (elt (_time_step . bv) 0) s)
              (write-long (if (>= _time_step 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _time_step s)(write-long (if (>= _time_step 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _time_step
#+(or :alpha :irix6 :x86_64)
      (setf _time_step (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _time_step (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
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

(setf (get jetbot_mini::Time_step_msg :md5sum-) "46804f01808531bde26c16e8419d8a77")
(setf (get jetbot_mini::Time_step_msg :datatype-) "jetbot_mini/Time_step_msg")
(setf (get jetbot_mini::Time_step_msg :definition-)
      "int64 time_step
")



(provide :jetbot_mini/Time_step_msg "46804f01808531bde26c16e8419d8a77")


