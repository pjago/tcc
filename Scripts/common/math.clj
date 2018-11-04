(ns common.math
  (:use [arcadia.core :exclude [log]] arcadia.linear)
  (:import [UnityEngine Mathf Vector3 Vector4 Matrix4x4 Quaternion Physics]
           [System Math Boolean Decimal Double Single Int16 UInt16 Int32 UInt32 Int64 UInt64]
           [System.IO TextWriter]
           [clojure.lang Ratio BigInt BigDecimal Sequential Counted IPersistentCollection APersistentVector IPersistentMap MapEntry])
  (:require [clojure.string :refer [capitalize]]))

(set! *unchecked-math* true) ;remember to uncoment on compile

;; ARITHMETIC

(def ^:const inf Double/PositiveInfinity)
(def ^:const nan Double/NaN)
(def ^:const eps Double/Epsilon)
(def ^:const finf Single/PositiveInfinity)
(def ^:const fnan Single/NaN)
(def ^:const feps Single/Epsilon)

(def ^:const operators
  (merge {:System.Math/unary '[abs acos asin atan ceiling cos cosh exp floor log log10 round sign sin sinh sqrt tan tanh truncate]
          :System.Math/binary '[atan2 pow]}
       #_{:UnityEngine.Mathf/unary '[abs acos asin atan ceil cos exp floor log round sign sin sqrt tan]
          :UnityEngine.Mathf/binary '[atan2 pow]}))

(defmacro do-defns []
  `(do ~@(mapcat #(let [src (-> % key namespace)
                        ary (-> % key name)]
                    (case ary
                      "unary"
                      (for [op (-> % val) 
                            :let [src-sym (symbol src (capitalize (str op)))]]
                        `(defn ~op [x#] (~src-sym (double x#))))
                      "binary"
                      (for [op (-> % val) 
                            :let [src-sym (symbol src (capitalize (str op)))]]
                        `(defn ~op [x# y#] (~src-sym (double x#) (double y#))))))
                    
                 operators)))

(do-defns)

;; GEOMETRIC

(defn eps= 
  ([x y] (eps= x y eps))
  ([x y eps] (-> (- x y) abs (< eps))))

(def ^:const pi Math/PI)
(def ^:const tau (* pi 2))
(def ^:const pi:2 (/ pi 2))
(def ^:const pi:3 (/ pi 3))
(def ^:const pi:4 (/ pi 4))
(def ^:const pi:6 (/ pi 6))
(def ^:const g 9.81)

(defn ll
  ([x] x)
  ([x y] (/ (* x y) (+ x y)))
  ([x y & more] (reduce ll (ll x y) more)))

;; UNITS

(def ^:const fps 60)
(def ^:const dt 0.02)
(def ^:const rpm.s:rad (/ 60 tau))
(def ^:const m.h:km.s 3.6)
(def ^:const deg:rad (/ 180 pi))
(def ^:const m:in 0.0254)
(def ^:const m:rad.in (/ 0.0254 tau))

;; BOUNDARY

(defn constrain "keeps number beetween interval"
  ([v val] (-> v (max (- val)) (min val)))
  ([v inf sup] (-> v (max inf) (min sup))))

(defn lmap [v o O p P] "maps number from an interval to another"
  (->> (- (+ o eps) O) / (* (- v o) (- p P)) (+ p)))

(defn lcycle "cycles number beetween interval"
  ([v val] (-> v (lmap (- val) val 0 1) (mod 1) (lmap 0 1 (- val) val)))
  ([v inf sup] (-> v (lmap inf sup 0 1) (mod 1) (lmap 0 1 inf sup))))