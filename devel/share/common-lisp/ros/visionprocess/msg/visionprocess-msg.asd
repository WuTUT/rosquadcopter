
(cl:in-package :asdf)

(defsystem "visionprocess-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ROI" :depends-on ("_package_ROI"))
    (:file "_package_ROI" :depends-on ("_package"))
  ))