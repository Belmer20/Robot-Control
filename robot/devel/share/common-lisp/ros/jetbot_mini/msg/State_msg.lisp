; Auto-generated. Do not edit!


(cl:in-package jetbot_mini-msg)


;//! \htmlinclude State_msg.msg.html

(cl:defclass <State_msg> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type cl:integer
    :initform 0)
   (ready
    :reader ready
    :initarg :ready
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass State_msg (<State_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <State_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'State_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jetbot_mini-msg:<State_msg> is deprecated: use jetbot_mini-msg:State_msg instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <State_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetbot_mini-msg:index-val is deprecated.  Use jetbot_mini-msg:index instead.")
  (index m))

(cl:ensure-generic-function 'ready-val :lambda-list '(m))
(cl:defmethod ready-val ((m <State_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetbot_mini-msg:ready-val is deprecated.  Use jetbot_mini-msg:ready instead.")
  (ready m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <State_msg>) ostream)
  "Serializes a message object of type '<State_msg>"
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ready) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <State_msg>) istream)
  "Deserializes a message object of type '<State_msg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'ready) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<State_msg>)))
  "Returns string type for a message object of type '<State_msg>"
  "jetbot_mini/State_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'State_msg)))
  "Returns string type for a message object of type 'State_msg"
  "jetbot_mini/State_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<State_msg>)))
  "Returns md5sum for a message object of type '<State_msg>"
  "9d2833016bdbb0ca11d012b3bd7a19bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'State_msg)))
  "Returns md5sum for a message object of type 'State_msg"
  "9d2833016bdbb0ca11d012b3bd7a19bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<State_msg>)))
  "Returns full string definition for message of type '<State_msg>"
  (cl:format cl:nil "int32 index~%bool ready~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'State_msg)))
  "Returns full string definition for message of type 'State_msg"
  (cl:format cl:nil "int32 index~%bool ready~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <State_msg>))
  (cl:+ 0
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <State_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'State_msg
    (cl:cons ':index (index msg))
    (cl:cons ':ready (ready msg))
))
