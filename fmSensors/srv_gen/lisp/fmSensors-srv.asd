
(cl:in-package :asdf)

(defsystem "fmSensors-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GTPS" :depends-on ("_package_GTPS"))
    (:file "_package_GTPS" :depends-on ("_package"))
  ))