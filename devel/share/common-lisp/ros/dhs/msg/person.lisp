; Auto-generated. Do not edit!


(cl:in-package dhs-msg)


;//! \htmlinclude person.msg.html

(cl:defclass <person> (roslisp-msg-protocol:ros-message)
  ((upper_red
    :reader upper_red
    :initarg :upper_red
    :type cl:fixnum
    :initform 0)
   (upper_green
    :reader upper_green
    :initarg :upper_green
    :type cl:fixnum
    :initform 0)
   (upper_blue
    :reader upper_blue
    :initarg :upper_blue
    :type cl:fixnum
    :initform 0)
   (lower_red
    :reader lower_red
    :initarg :lower_red
    :type cl:fixnum
    :initform 0)
   (lower_green
    :reader lower_green
    :initarg :lower_green
    :type cl:fixnum
    :initform 0)
   (lower_blue
    :reader lower_blue
    :initarg :lower_blue
    :type cl:fixnum
    :initform 0))
)

(cl:defclass person (<person>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <person>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'person)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dhs-msg:<person> is deprecated: use dhs-msg:person instead.")))

(cl:ensure-generic-function 'upper_red-val :lambda-list '(m))
(cl:defmethod upper_red-val ((m <person>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dhs-msg:upper_red-val is deprecated.  Use dhs-msg:upper_red instead.")
  (upper_red m))

(cl:ensure-generic-function 'upper_green-val :lambda-list '(m))
(cl:defmethod upper_green-val ((m <person>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dhs-msg:upper_green-val is deprecated.  Use dhs-msg:upper_green instead.")
  (upper_green m))

(cl:ensure-generic-function 'upper_blue-val :lambda-list '(m))
(cl:defmethod upper_blue-val ((m <person>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dhs-msg:upper_blue-val is deprecated.  Use dhs-msg:upper_blue instead.")
  (upper_blue m))

(cl:ensure-generic-function 'lower_red-val :lambda-list '(m))
(cl:defmethod lower_red-val ((m <person>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dhs-msg:lower_red-val is deprecated.  Use dhs-msg:lower_red instead.")
  (lower_red m))

(cl:ensure-generic-function 'lower_green-val :lambda-list '(m))
(cl:defmethod lower_green-val ((m <person>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dhs-msg:lower_green-val is deprecated.  Use dhs-msg:lower_green instead.")
  (lower_green m))

(cl:ensure-generic-function 'lower_blue-val :lambda-list '(m))
(cl:defmethod lower_blue-val ((m <person>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dhs-msg:lower_blue-val is deprecated.  Use dhs-msg:lower_blue instead.")
  (lower_blue m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <person>) ostream)
  "Serializes a message object of type '<person>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'upper_red)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'upper_green)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'upper_blue)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lower_red)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lower_green)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lower_blue)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <person>) istream)
  "Deserializes a message object of type '<person>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'upper_red)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'upper_green)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'upper_blue)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lower_red)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lower_green)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lower_blue)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<person>)))
  "Returns string type for a message object of type '<person>"
  "dhs/person")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'person)))
  "Returns string type for a message object of type 'person"
  "dhs/person")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<person>)))
  "Returns md5sum for a message object of type '<person>"
  "b1c29f70675460405e64a004cc34ba02")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'person)))
  "Returns md5sum for a message object of type 'person"
  "b1c29f70675460405e64a004cc34ba02")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<person>)))
  "Returns full string definition for message of type '<person>"
  (cl:format cl:nil "uint8 upper_red~%uint8 upper_green~%uint8 upper_blue~%uint8 lower_red~%uint8 lower_green~%uint8 lower_blue~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'person)))
  "Returns full string definition for message of type 'person"
  (cl:format cl:nil "uint8 upper_red~%uint8 upper_green~%uint8 upper_blue~%uint8 lower_red~%uint8 lower_green~%uint8 lower_blue~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <person>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <person>))
  "Converts a ROS message object to a list"
  (cl:list 'person
    (cl:cons ':upper_red (upper_red msg))
    (cl:cons ':upper_green (upper_green msg))
    (cl:cons ':upper_blue (upper_blue msg))
    (cl:cons ':lower_red (lower_red msg))
    (cl:cons ':lower_green (lower_green msg))
    (cl:cons ':lower_blue (lower_blue msg))
))
