; Auto-generated. Do not edit!


(cl:in-package jetbot_mini-msg)


;//! \htmlinclude Commands_msg.msg.html

(cl:defclass <Commands_msg> (roslisp-msg-protocol:ros-message)
  ((action
    :reader action
    :initarg :action
    :type cl:string
    :initform "")
   (subject
    :reader subject
    :initarg :subject
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0)
   (unit
    :reader unit
    :initarg :unit
    :type cl:string
    :initform ""))
)

(cl:defclass Commands_msg (<Commands_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Commands_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Commands_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jetbot_mini-msg:<Commands_msg> is deprecated: use jetbot_mini-msg:Commands_msg instead.")))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <Commands_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetbot_mini-msg:action-val is deprecated.  Use jetbot_mini-msg:action instead.")
  (action m))

(cl:ensure-generic-function 'subject-val :lambda-list '(m))
(cl:defmethod subject-val ((m <Commands_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetbot_mini-msg:subject-val is deprecated.  Use jetbot_mini-msg:subject instead.")
  (subject m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Commands_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetbot_mini-msg:value-val is deprecated.  Use jetbot_mini-msg:value instead.")
  (value m))

(cl:ensure-generic-function 'unit-val :lambda-list '(m))
(cl:defmethod unit-val ((m <Commands_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetbot_mini-msg:unit-val is deprecated.  Use jetbot_mini-msg:unit instead.")
  (unit m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Commands_msg>) ostream)
  "Serializes a message object of type '<Commands_msg>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'action))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'action))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'subject))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'subject))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'unit))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'unit))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Commands_msg>) istream)
  "Deserializes a message object of type '<Commands_msg>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'action) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'subject) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'subject) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'unit) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'unit) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Commands_msg>)))
  "Returns string type for a message object of type '<Commands_msg>"
  "jetbot_mini/Commands_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Commands_msg)))
  "Returns string type for a message object of type 'Commands_msg"
  "jetbot_mini/Commands_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Commands_msg>)))
  "Returns md5sum for a message object of type '<Commands_msg>"
  "88c97c65bce824d2b08b3ca7fe1f5632")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Commands_msg)))
  "Returns md5sum for a message object of type 'Commands_msg"
  "88c97c65bce824d2b08b3ca7fe1f5632")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Commands_msg>)))
  "Returns full string definition for message of type '<Commands_msg>"
  (cl:format cl:nil "string action~%string subject~%float64 value~%string unit~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Commands_msg)))
  "Returns full string definition for message of type 'Commands_msg"
  (cl:format cl:nil "string action~%string subject~%float64 value~%string unit~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Commands_msg>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'action))
     4 (cl:length (cl:slot-value msg 'subject))
     8
     4 (cl:length (cl:slot-value msg 'unit))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Commands_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'Commands_msg
    (cl:cons ':action (action msg))
    (cl:cons ':subject (subject msg))
    (cl:cons ':value (value msg))
    (cl:cons ':unit (unit msg))
))
