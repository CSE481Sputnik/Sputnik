; Auto-generated. Do not edit!


(cl:in-package learning_actionlib-msg)


;//! \htmlinclude MapSumFeedback.msg.html

(cl:defclass <MapSumFeedback> (roslisp-msg-protocol:ros-message)
  ((res
    :reader res
    :initarg :res
    :type cl:integer
    :initform 0))
)

(cl:defclass MapSumFeedback (<MapSumFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MapSumFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MapSumFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name learning_actionlib-msg:<MapSumFeedback> is deprecated: use learning_actionlib-msg:MapSumFeedback instead.")))

(cl:ensure-generic-function 'res-val :lambda-list '(m))
(cl:defmethod res-val ((m <MapSumFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader learning_actionlib-msg:res-val is deprecated.  Use learning_actionlib-msg:res instead.")
  (res m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MapSumFeedback>) ostream)
  "Serializes a message object of type '<MapSumFeedback>"
  (cl:let* ((signed (cl:slot-value msg 'res)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MapSumFeedback>) istream)
  "Deserializes a message object of type '<MapSumFeedback>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'res) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MapSumFeedback>)))
  "Returns string type for a message object of type '<MapSumFeedback>"
  "learning_actionlib/MapSumFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MapSumFeedback)))
  "Returns string type for a message object of type 'MapSumFeedback"
  "learning_actionlib/MapSumFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MapSumFeedback>)))
  "Returns md5sum for a message object of type '<MapSumFeedback>"
  "ca16cfbd5443ad97f6cc7ffd6bb67292")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MapSumFeedback)))
  "Returns md5sum for a message object of type 'MapSumFeedback"
  "ca16cfbd5443ad97f6cc7ffd6bb67292")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MapSumFeedback>)))
  "Returns full string definition for message of type '<MapSumFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%int32 res~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MapSumFeedback)))
  "Returns full string definition for message of type 'MapSumFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%int32 res~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MapSumFeedback>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MapSumFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'MapSumFeedback
    (cl:cons ':res (res msg))
))
