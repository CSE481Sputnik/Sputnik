; Auto-generated. Do not edit!


(cl:in-package learning_actionlib-msg)


;//! \htmlinclude GripperResult.msg.html

(cl:defclass <GripperResult> (roslisp-msg-protocol:ros-message)
  ((v
    :reader v
    :initarg :v
    :type cl:integer
    :initform 0)
   (p
    :reader p
    :initarg :p
    :type cl:integer
    :initform 0))
)

(cl:defclass GripperResult (<GripperResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name learning_actionlib-msg:<GripperResult> is deprecated: use learning_actionlib-msg:GripperResult instead.")))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <GripperResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader learning_actionlib-msg:v-val is deprecated.  Use learning_actionlib-msg:v instead.")
  (v m))

(cl:ensure-generic-function 'p-val :lambda-list '(m))
(cl:defmethod p-val ((m <GripperResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader learning_actionlib-msg:p-val is deprecated.  Use learning_actionlib-msg:p instead.")
  (p m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperResult>) ostream)
  "Serializes a message object of type '<GripperResult>"
  (cl:let* ((signed (cl:slot-value msg 'v)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'p)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperResult>) istream)
  "Deserializes a message object of type '<GripperResult>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'v) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'p) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperResult>)))
  "Returns string type for a message object of type '<GripperResult>"
  "learning_actionlib/GripperResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperResult)))
  "Returns string type for a message object of type 'GripperResult"
  "learning_actionlib/GripperResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperResult>)))
  "Returns md5sum for a message object of type '<GripperResult>"
  "6f3f92964da9d19360d5496061fe5eff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperResult)))
  "Returns md5sum for a message object of type 'GripperResult"
  "6f3f92964da9d19360d5496061fe5eff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperResult>)))
  "Returns full string definition for message of type '<GripperResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%int64 v~%int64 p~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperResult)))
  "Returns full string definition for message of type 'GripperResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%int64 v~%int64 p~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperResult>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperResult>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperResult
    (cl:cons ':v (v msg))
    (cl:cons ':p (p msg))
))
