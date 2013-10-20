
(cl:in-package :asdf)

(defsystem "dhs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "person" :depends-on ("_package_person"))
    (:file "_package_person" :depends-on ("_package"))
    (:file "bag" :depends-on ("_package_bag"))
    (:file "_package_bag" :depends-on ("_package"))
  ))