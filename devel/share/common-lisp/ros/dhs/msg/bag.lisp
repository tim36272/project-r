; Auto-generated. Do not edit!


(cl:in-package dhs-msg)


;//! \htmlinclude bag.msg.html

(cl:defclass <bag> (roslisp-msg-protocol:ros-message)
  ((blue
    :reader blue
    :initarg :blue
    :type cl:fixnum
    :initform 0)
   (green
    :reader green
    :initarg :green
    :type cl:fixnum
    :initform 0)
   (red
    :reader red
    :initarg :red
    :type cl:fixnum
    :initform 0))
)

(cl:defclass bag (<bag>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <bag>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'bag)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dhs-msg:<bag> is deprecated: use dhs-msg:bag instead.")))

(cl:ensure-generic-function 'blue-val :lambda-list '(m))
(cl:defmethod blue-val ((m <bag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dhs-msg:blue-val is deprecated.  Use dhs-msg:blue instead.")
  (blue m))

(cl:ensure-generic-function 'green-val :lambda-list '(m))
(cl:defmethod green-val ((m <bag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dhs-msg:green-val is deprecated.  Use dhs-msg:green instead.")
  (green m))

(cl:ensure-generic-function 'red-val :lambda-list '(m))
(cl:defmethod red-val ((m <bag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dhs-msg:red-val is deprecated.  Use dhs-msg:red instead.")
  (red m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <bag>) ostream)
  "Serializes a message object of type '<bag>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'blue)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'green)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'red)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <bag>) istream)
  "Deserializes a message object of type '<bag>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'blue)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'green)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'red)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<bag>)))
  "Returns string type for a message object of type '<bag>"
  "dhs/bag")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'bag)))
  "Returns string type for a message object of type 'bag"
  "dhs/bag")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<bag>)))
  "Returns md5sum for a message object of type '<bag>"
  "ecb8956a75edf997e18aad34e20d469b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'bag)))
  "Returns md5sum for a message object of type 'bag"
  "ecb8956a75edf997e18aad34e20d469b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<bag>)))
  "Returns full string definition for message of type '<bag>"
  (cl:format cl:nil "uint8 blue~%uint8 green~%uint8 red~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'bag)))
  "Returns full string definition for message of type 'bag"
  (cl:format cl:nil "uint8 blue~%uint8 green~%uint8 red~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <bag>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <bag>))
  "Converts a ROS message object to a list"
  (cl:list 'bag
    (cl:cons ':blue (blue msg))
    (cl:cons ':green (green msg))
    (cl:cons ':red (red msg))
))
