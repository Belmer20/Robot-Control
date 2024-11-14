; Auto-generated. Do not edit!


(cl:in-package jetbot_mini-msg)


;//! \htmlinclude Position_msg.msg.html

(cl:defclass <Position_msg> (roslisp-msg-protocol:ros-message)
  ((robot_index
    :reader robot_index
    :initarg :robot_index
    :type cl:integer
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0))
)

(cl:defclass Position_msg (<Position_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Position_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Position_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jetbot_mini-msg:<Position_msg> is deprecated: use jetbot_mini-msg:Position_msg instead.")))

(cl:ensure-generic-function 'robot_index-val :lambda-list '(m))
(cl:defmethod robot_index-val ((m <Position_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetbot_mini-msg:robot_index-val is deprecated.  Use jetbot_mini-msg:robot_index instead.")
  (robot_index m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Position_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetbot_mini-msg:x-val is deprecated.  Use jetbot_mini-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Position_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetbot_mini-msg:y-val is deprecated.  Use jetbot_mini-msg:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Position_msg>) ostream)
  "Serializes a message object of type '<Position_msg>"
  (cl:let* ((signed (cl:slot-value msg 'robot_index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Position_msg>) istream)
  "Deserializes a message object of type '<Position_msg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Position_msg>)))
  "Returns string type for a message object of type '<Position_msg>"
  "jetbot_mini/Position_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Position_msg)))
  "Returns string type for a message object of type 'Position_msg"
  "jetbot_mini/Position_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Position_msg>)))
  "Returns md5sum for a message object of type '<Position_msg>"
  "c8685de48e63ca6aa63f0d3bc9a2af72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Position_msg)))
  "Returns md5sum for a message object of type 'Position_msg"
  "c8685de48e63ca6aa63f0d3bc9a2af72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Position_msg>)))
  "Returns full string definition for message of type '<Position_msg>"
  (cl:format cl:nil "int32 robot_index~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Position_msg)))
  "Returns full string definition for message of type 'Position_msg"
  (cl:format cl:nil "int32 robot_index~%float64 x~%float64 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Position_msg>))
  (cl:+ 0
     4
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Position_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'Position_msg
    (cl:cons ':robot_index (robot_index msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
