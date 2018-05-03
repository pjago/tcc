(ns amalloy.ring-buffer
  (:use arcadia.core)
  (:import (clojure.lang Counted
                         Sequential
                         IPersistentCollection
                         IPersistentStack
                         IPersistentMap
                         Indexed
                         Associative
                         Reversible
                         IObj)
           (System.IO TextWriter)))

;; If one of our numbers gets over 2 billion, the user's ring buffer is way too large!
;; and count is defined to return an int anyway, so we can't make it work regardless.
;; So we'll just skip that overflow check for a mild speed boost.
(set! *unchecked-math* true)

(defmutable RingBuffer [^long start ^long len buf meta]
  Counted
  (count [this] len)
  Sequential ;; tagging interface
  IObj
  (withMeta [this m]
    (->RingBuffer start len buf m))
  (meta [this] meta)
  Object
  (ToString [this]
    (pr-str (lazy-seq (seq this))))
  IPersistentStack
  (peek [this] ;change this so it returns the latest
    (nth buf (mod (dec start) len)))
  (pop [this] ;change this so it pops the latest
    (if-not (zero? len)
      (->RingBuffer start (dec len) (assoc buf (mod (dec start) len) nil) meta)))
  (empty [this]
    (->RingBuffer 0 0 (vec (repeat (count buf) nil)) meta))
  (equiv [this other]
    (and (sequential? other)
         (or (not (counted? other))
             (= (count this) (count other)))
         (= (seq this) (seq other))))
  IPersistentCollection
  (cons [this x]
    (if (= len (count buf))
      (->RingBuffer (rem (inc start) len) len (assoc buf start x) meta)
      (->RingBuffer start (inc len) (assoc buf (rem (+ start len) (count buf)) x) meta)))
  (seq [this] 
    (seq (for [i (range len)]
           (nth buf (rem (+ start i) (count buf))))))
  Reversible
  (rseq [this]
    (seq (for [i (range (- len 1) -1 -1)]
           (nth buf (rem (+ start i) (count buf))))))
  Indexed ;add this
  (nth [this i]
    (nth buf (mod (+ start i) len)))
  (nth [this i default]
    (if (< (max i (- i)) len)
      (nth buf (mod (+ start i) len))
      default)))

; (defmethod print-method RingBuffer [^RingBuffer b ^TextWriter w]
;   (.Write w "#amalloy/ring-buffer ")
;   (print-method [(count (.buf b)) (sequence b)] w))

; (defn- read-method [[capacity items]] ;TODO: fix this so it serializes
;   (->RingBuffer 0 (count items) (vec (take capacity (concat items (repeat nil)))) nil))

(defn ring-buffer
  "Create an empty ring buffer with the specified [capacity]."
  [capacity]
  (->RingBuffer 0 0 (vec (repeat capacity nil)) nil))

;; Arcadia # 268

; (alter-var-root #'*data-readers* assoc 'amalloy/ring-buffer #'read-method)

; ;; and we also have to do this, for the repl:
; (when (.getThreadBinding ^clojure.lang.Var #'*data-readers*)
;   (set! clojure.core/*data-readers*
;     (merge clojure.core/*data-readers*
;       ;; I guess. so weird
;       (.getRawRoot #'clojure.core/*data-readers*))))
;       ;;'arcadia.core/mutable #'parse-user-type

; ;; ============================================================

; ;; this is stupid

; (def the-bucket (.getRawRoot #'*data-readers*))