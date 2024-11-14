; Auto-generated. Do not edit!


(cl:in-package jetbot_mini-msg)


;//! \htmlinclude Command_classes_msg.msg.html

(cl:defclass <Command_classes_msg> (roslisp-msg-protocol:ros-message)
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
    :type cl:string
    :initform "")
   (unit
    :reader unit
    :initarg :unit
    :type cl:string
    :initform ""))
)

(cl:defclass Command_classes_msg (<Command_classes_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Command_classes_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Command_classes_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jetbot_mini-msg:<Command_classes_msg> is deprecated: use jetbot_mini-msg:Command_classes_msg instead.")))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <Command_classes_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetbot_mini-msg:action-val is deprecated.  Use jetbot_mini-msg:action instead.")
  (action m))

(cl:ensure-generic-function 'subject-val :lambda-list '(m))
(cl:defmethod subject-val ((m <Command_classes_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetbot_mini-msg:subject-val is deprecated.  Use jetbot_mini-msg:subject instead.")
  (subject m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Command_classes_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetbot_mini-msg:value-val is deprecated.  Use jetbot_mini-msg:value instead.")
  (value m))

(cl:ensure-generic-function 'unit-val :lambda-list '(m))
(cl:defmethod unit-val ((m <Command_classes_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetbot_mini-msg:unit-val is deprecated.  Use jetbot_mini-msg:unit instead.")
  (unit m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Command_classes_msg>) ostream)
  "Serializes a message object of type '<Command_classes_msg>"
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'value))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'unit))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'unit))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Command_classes_msg>) istream)
  "Deserializes a message object of type '<Command_classes_msg>"
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
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'value) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Command_classes_msg>)))
  "Returns string type for a message object of type '<Command_classes_msg>"
  "jetbot_mini/Command_classes_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Command_classes_msg)))
  "Returns string type for a message object of type 'Command_classes_msg"
  "jetbot_mini/Command_classes_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Command_classes_msg>)))
  "Returns md5sum for a message object of type '<Command_classes_msg>"
  "06f8489ff1e7fcaba60778ad9099d7c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Command_classes_msg)))
  "Returns md5sum for a message object of type 'Command_classes_msg"
  "06f8489ff1e7fcaba60778ad9099d7c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Command_classes_msg>)))
  "Returns full string definition for message of type '<Command_classes_msg>"
  (cl:format cl:nil "string action~%string subject~%string value~%string unit~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Command_classes_msg)))
  "Returns full string definition for message of type 'Command_classes_msg"
  (cl:format cl:nil "string action~%string subject~%string value~%string unit~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Command_classes_msg>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'action))
     4 (cl:length (cl:slot-value msg 'subject))
     4 (cl:length (cl:slot-value msg 'value))
     4 (cl:length (cl:slot-value msg 'unit))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Command_classes_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'Command_classes_msg
    (cl:cons ':action (action msg))
    (cl:cons ':subject (subject msg))
    (cl:cons ':value (value msg))
    (cl:cons ':unit (unit msg))
))
