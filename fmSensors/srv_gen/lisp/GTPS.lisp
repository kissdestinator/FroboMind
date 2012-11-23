; Auto-generated. Do not edit!


(cl:in-package fmSensors-srv)


;//! \htmlinclude GTPS-request.msg.html

(cl:defclass <GTPS-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GTPS-request (<GTPS-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GTPS-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GTPS-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fmSensors-srv:<GTPS-request> is deprecated: use fmSensors-srv:GTPS-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GTPS-request>) ostream)
  "Serializes a message object of type '<GTPS-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GTPS-request>) istream)
  "Deserializes a message object of type '<GTPS-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GTPS-request>)))
  "Returns string type for a service object of type '<GTPS-request>"
  "fmSensors/GTPSRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GTPS-request)))
  "Returns string type for a service object of type 'GTPS-request"
  "fmSensors/GTPSRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GTPS-request>)))
  "Returns md5sum for a message object of type '<GTPS-request>"
  "8c7b89222eea218aa26cd807453bd8e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GTPS-request)))
  "Returns md5sum for a message object of type 'GTPS-request"
  "8c7b89222eea218aa26cd807453bd8e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GTPS-request>)))
  "Returns full string definition for message of type '<GTPS-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GTPS-request)))
  "Returns full string definition for message of type 'GTPS-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GTPS-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GTPS-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GTPS-request
))
;//! \htmlinclude GTPS-response.msg.html

(cl:defclass <GTPS-response> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0)
   (date
    :reader date
    :initarg :date
    :type cl:real
    :initform 0))
)

(cl:defclass GTPS-response (<GTPS-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GTPS-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GTPS-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fmSensors-srv:<GTPS-response> is deprecated: use fmSensors-srv:GTPS-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <GTPS-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fmSensors-srv:x-val is deprecated.  Use fmSensors-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <GTPS-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fmSensors-srv:y-val is deprecated.  Use fmSensors-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'date-val :lambda-list '(m))
(cl:defmethod date-val ((m <GTPS-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fmSensors-srv:date-val is deprecated.  Use fmSensors-srv:date instead.")
  (date m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GTPS-response>) ostream)
  "Serializes a message object of type '<GTPS-response>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'date)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'date) (cl:floor (cl:slot-value msg 'date)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GTPS-response>) istream)
  "Deserializes a message object of type '<GTPS-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'date) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GTPS-response>)))
  "Returns string type for a service object of type '<GTPS-response>"
  "fmSensors/GTPSResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GTPS-response)))
  "Returns string type for a service object of type 'GTPS-response"
  "fmSensors/GTPSResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GTPS-response>)))
  "Returns md5sum for a message object of type '<GTPS-response>"
  "8c7b89222eea218aa26cd807453bd8e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GTPS-response)))
  "Returns md5sum for a message object of type 'GTPS-response"
  "8c7b89222eea218aa26cd807453bd8e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GTPS-response>)))
  "Returns full string definition for message of type '<GTPS-response>"
  (cl:format cl:nil "int32 x~%int32 y~%time date~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GTPS-response)))
  "Returns full string definition for message of type 'GTPS-response"
  (cl:format cl:nil "int32 x~%int32 y~%time date~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GTPS-response>))
  (cl:+ 0
     4
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GTPS-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GTPS-response
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':date (date msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GTPS)))
  'GTPS-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GTPS)))
  'GTPS-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GTPS)))
  "Returns string type for a service object of type '<GTPS>"
  "fmSensors/GTPS")