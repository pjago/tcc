(ns pjago.test
  (:use arcadia.core arcadia.linear)
  (:require [common.processing :as x])
  (:import [UnityEngine GameObject Resources Screen Time GUI Rect]))

(def wait 5) ;seconds
(def last-obj (atom (x/resource :sansbox GameObject)))
(def ptime (atom 0))

(defn main [^GameObject gob _]
  (let [time (int (mod Time/time wait))
        test (and (zero? time) (not (zero? @ptime)))]
    (reset! ptime time)
    (when test
      (let [q (x/clone! (x/resource :sansbox GameObject))]
        (reset! last-obj q)
        (log (gensym "Creating new gob "))
        (set! (.. q transform position) (v3 0 5 0))
        (set! (.. q transform rotation) (aa 1 1 0 0))
        (destroy q wait)))))

(defn info [^GameObject gob k]
  (let [H (/ Screen/height 2)
        W (/ Screen/width 2)]
    (GUI/Label (Rect. (- W 20) (- H 20) (+ W 10) (+ H 10)) 
               (str (or @last-obj "NOT FOUND")))))