; Auto-generated. Do not edit!


(cl:in-package learning_actionlib-msg)


;//! \htmlinclude GripperGoal.msg.html

(cl:defclass <GripperGoal> (roslisp-msg-protocol:ros-message)
  ((p
    :reader p
    :initarg :p
    :type cl:integer
    :initform 0))
)

(cl:defclass GripperGoal (<GripperGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name learning_actionlib-msg:<GripperGoal> is deprecated: use learning_actionlib-msg:GripperGoal instead.")))

(cl:ensure-generic-function 'p-val :lambda-list '(m))
(cl:defmethod p-val ((m <GripperGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader learning_actionlib-msg:p-val is deprecated.  Use learning_actionlib-msg:p instead.")
  (p m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperGoal>) ostream)
  "Serializes a message object of type '<GripperGoal>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperGoal>) istream)
  "Deserializes a message object of type '<GripperGoal>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperGoal>)))
  "Returns string type for a message object of type '<GripperGoal>"
  "learning_actionlib/GripperGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperGoal)))
  "Returns string type for a message object of type 'GripperGoal"
  "learning_actionlib/GripperGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperGoal>)))
  "Returns md5sum for a message object of type '<GripperGoal>"
  "fc79ae4c06a8a778f1823c93d70f35d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperGoal)))
  "Returns md5sum for a message object of type 'GripperGoal"
  "fc79ae4c06a8a778f1823c93d70f35d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperGoal>)))
  "Returns full string definition for message of type '<GripperGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%int64 p~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperGoal)))
  "Returns full string definition for message of type 'GripperGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%int64 p~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperGoal>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperGoal
    (cl:cons ':p (p msg))
))