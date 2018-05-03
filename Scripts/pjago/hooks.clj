(ns pjago.hooks
  (:use arcadia.core arcadia.linear)
  (:import [UnityEngine GameObject Transform Vector3 Rigidbody Joint SpringJoint FixedJoint HingeJoint ConfigurableJoint Camera])
  (:require [common.math :as m]))

;; HOOKS

(defmutable LockPos [^GameObject lock ^Vector3 offset])

(defn follow-pos [^GameObject gob k]
  (let [igob ^LockPos (state gob k)]
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
    (child- pgob gob true))))

;; UTIL
          
(defn mario-cam! [^GameObject cam ^GameObject mario]
  (let [off (v3 0 0 15)]
    (hook+ cam :fixed-update :follow #'follow-pos)
    (state+ cam :follow (->LockPos mario off))
    (set! (.. cam -transform -position)
          (v3+ (.. mario -transform -position)
               off))
    cam))

(defn mirror! [gobs ^Vector3 point ^Vector3 axis]
  (let [n (count gobs)
        span (- 180 (/ 180 n))
        roll (m/lmap (range n) 0 (dec n) span (- span))]
    (mapv #(.RotateAround (.-transform %1) point axis %2)
          gobs
          roll)
    gobs))