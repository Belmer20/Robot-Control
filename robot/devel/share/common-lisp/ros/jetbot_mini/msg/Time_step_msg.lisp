; Auto-generated. Do not edit!


(cl:in-package jetbot_mini-msg)


;//! \htmlinclude Time_step_msg.msg.html

(cl:defclass <Time_step_msg> (roslisp-msg-protocol:ros-message)
  ((time_step
    :reader time_step
    :initarg :time_step
    :type cl:integer
    :initform 0))
)

(cl:defclass Time_step_msg (<Time_step_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Time_step_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Time_step_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jetbot_mini-msg:<Time_step_msg> is deprecated: use jetbot_mini-msg:Time_step_msg instead.")))

(cl:ensure-generic-function 'time_step-val :lambda-list '(m))
(cl:defmethod time_step-val ((m <Time_step_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetbot_mini-msg:time_step-val is deprecated.  Use jetbot_mini-msg:time_step instead.")
  (time_step m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Time_step_msg>) ostream)
  "Serializes a message object of type '<Time_step_msg>"
  (cl:let* ((signed (cl:slot-value msg 'time_step)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Time_step_msg>) istream)
  "Deserializes a message object of type '<Time_step_msg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time_step) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Time_step_msg>)))
  "Returns string type for a message object of type '<Time_step_msg>"
  "jetbot_mini/Time_step_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Time_step_msg)))
  "Returns string type for a message object of type 'Time_step_msg"
  "jetbot_mini/Time_step_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Time_step_msg>)))
  "Returns md5sum for a message object of type '<Time_step_msg>"
  "46804f01808531bde26c16e8419d8a77")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Time_step_msg)))
  "Returns md5sum for a message object of type 'Time_step_msg"
  "46804f01808531bde26c16e8419d8a77")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Time_step_msg>)))
  "Returns full string definition for message of type '<Time_step_msg>"
  (cl:format cl:nil "int64 time_step~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Time_step_msg)))
  "Returns full string definition for message of type 'Time_step_msg"
  (cl:format cl:nil "int64 time_step~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Time_step_msg>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Time_step_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'Time_step_msg
    (cl:cons ':time_step (time_step msg))
))
