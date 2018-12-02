(ns pjago.hooks
  (:use arcadia.core arcadia.linear)
  (:import [UnityEngine GameObject Transform Quaternion Vector3 Rigidbody Joint SpringJoint FixedJoint HingeJoint ConfigurableJoint Camera RigidbodyConstraints])
  (:require [common.math :as m]))

;; HOOKS

(defmutable LockPos [^GameObject lock ^Vector3 offset])

(defn follow-pos [^GameObject gob k]
  (if-let [igob ^LockPos (state gob k)]
    (if-let [pgob (obj-nil (.-lock igob))]
      (with-cmpt gob [tr Transform]
        (set! (.-position tr)
              (v3+ (.. pgob -transform -position)
                   (.-offset igob)))))))

(def joint-map
  {:fixed FixedJoint
   :hinge HingeJoint
   :configurable ConfigurableJoint
   :spring SpringJoint})

(defn snap-joint
 ([^GameObject gob k]
  (if-cmpt (.. gob transform root) [prb Rigidbody]
    (if-not (= (gobj prb) gob)
      (let [jt (get joint-map (state gob k) FixedJoint)
            jo (ensure-cmpt gob jt)]
        (set! (.-connectedBody jo) prb)))))
 ([^GameObject gob k break-force]
  (if-let [pgob (parent gob)]
    (child- pgob gob)))); true))))

;; UTIL

;idempotency is weird ;@bug
(defn mario-cam! 
  ([^GameObject cam]
   (hook+ cam :late-update :follow #'follow-pos)
   (state+ cam :follow (->LockPos nil (.. cam transform position))))
  ([^GameObject cam ^GameObject mario]
   (let [offset (state cam :focus)]
     (state+ cam :follow (->LockPos mario offset))
     (set! (.. cam transform position) ;let's update it once before follow-pos
           (v3+ (.. mario transform position)
                offset))
     cam)))

(defn mirror! [gobs ^Vector3 point ^Vector3 axis]
  (let [n (count gobs)
        span (- 180 (/ 180 n))
        roll (mapv #(m/lmap % 0 (dec n) span (- span)) (range n))]
    (mapv #(.RotateAround (.-transform %1) point axis %2)
          gobs
          roll)
    gobs))

;not good, use processing/freeze
(defn only-pitch [^GameObject gob k]
  (let [tr (cmpt gob Transform)
        pitch (.. gob transform rotation eulerAngles x)]
    (set! (.. gob transform position) (v3 0 0.5 0))
    (set! (.. gob transform localRotation) 
          (Quaternion/Euler (v3 pitch 0 0)))))
  
  
  