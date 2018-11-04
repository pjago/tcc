(ns common.math
  (:refer-clojure :exclude [+ - * / mod quot min max defn defmethod])
  (:use ;[magic.api :only [defn faster]]
        [arcadia.core :exclude [log]] arcadia.linear)
  (:import [UnityEngine Mathf Vector3 Vector4 Matrix4x4 Quaternion Physics]
           [System Math Boolean Decimal Double Single Int16 UInt16 Int32 UInt32 Int64 UInt64]
           [System.IO TextWriter]
           [clojure.lang Ratio BigInt BigDecimal Sequential Counted IPersistentCollection APersistentVector IPersistentMap MapEntry])
  (:require [clojure.string :refer [capitalize ends-with?]]))

(set! *unchecked-math* true) ;remember to uncoment on compile

(def ^:dynamic *h* (make-hierarchy))

(deftype Complex [x y]
  Object
  (ToString [this]
    (if (instance? Complex y)
      (str x "+j(" y")")
      (str x "+j" y))))

(clojure.core/defmethod print-method Complex [v ^TextWriter w]
  (.Write w (.ToString v)))

(def ^:const number ::number)
(def ^:const cnumber ::cnumber) ;todo: include BigInts
(def ^:const number+ ::number+)
(def ^:const dotag? (set '[Complex Double Int64 Vector3 Vector4 Matrix4x4])) ;for clojure
; (clojure.core/defn dotag? [x] (not (or (keyword? x) (= Ratio x))))
(def ^:const no-magic true) ;for magic
(def ^:const camel? (set '[UnityEngine.Mathf System.Math]))

;maybe magic has some trouble with derive...
(alter-var-root #'*h*
  #(-> (reduce (fn [acc in] (derive acc in ::number)) %
               [Double Single Int64 Int32])
       (derive ::number ::cnumber)
       (derive Ratio ::cnumber)
       (derive ::cnumber ::number+)
       (derive Complex ::number+)
       (derive Object ::any)))

;how to use transducers (or eductions) to speed things up??
;sure, multimethods are flexible. but what about interfaces?

(clojure.core/defn unique-descendants [k] ;aka children
  (if (keyword? k)
    (transduce
      (mapcat unique-descendants)
      disj
      (descendants *h* k)
      (descendants *h* k))))  

(defmacro defmethod [multifn dispatch-val args & body]
  (letfn [(may-tag [x t]
            (if (dotag? t)
              (with-meta x {:tag (if (symbol? t) (resolve t) t)})
              x))
          (apply-tag [dispatch-val args]
            (condp = (count args)
             0 args
             1 [(may-tag (first args) dispatch-val)]
             2 (mapv may-tag args dispatch-val)
             :else args))
          (method [dispatch-val args]
            (if (or no-magic (:no-magic (meta args)))
              `(clojure.core/defmethod ~multifn ~dispatch-val ~(apply-tag dispatch-val args) ~@body)
              `(do (~'magic.api/defn f#
                     ~(apply-tag dispatch-val args)
                     ~@body)
                   (. ~multifn ~'addMethod ~dispatch-val f#))))
            ; `(clojure.core/defmethod ~multifn ~dispatch-val ~args
            ;   (~(if (:no-magic (meta args)) 'do 'magic.api/faster)
            ;    (let ~(vec (interleave
            ;                 (condp = (count args)
            ;                  0 args
            ;                  1 [(may-tag (first args) dispatch-val)]
            ;                  2 (mapv may-tag args dispatch-val)
            ;                  :else args)
            ;                 args))
            ;     ~@body)))) ;magic
          (dispatch [dispatch-val args]
            (if no-magic [dispatch-val]
              (if (pos? (dec (count args)))
                (->> (map #(dispatch % []) dispatch-val)
                     (reduce #(for [a %1 b %2] [a b]))
                     flatten
                     (partition (count args))
                     (map vec))
                (->> ;(unique-descendants dispatch-val)
                     (descendants *h* dispatch-val)
                     (remove keyword?)
                     (cons dispatch-val)
                     ; (remove keyword?)
                     (#(if (= ::any dispatch-val) (rest %) %))))))
          (methods [dispatch-val args]
            (map method (dispatch dispatch-val args) (repeat args)))]
    (if-not (and (= 2 (count args)) (:com (meta args)))
      `(do ~@(methods dispatch-val args))
      `(do ~@(methods dispatch-val args)
           ~@(methods (vec (rseq dispatch-val)) (vec (rseq args)))))))

;; GENERIC

(def ^:const any ::any)
(def ^:const nulary nil)
(def ^:const nary ::nary)

(clojure.core/defn unary [x] (type x))
(clojure.core/defn binary [x y] (vector (type x) (type y)))
(clojure.core/defn multiary ;magic: multiarity won't work
  ([] nulary)
  ([x] (unary x))
  ([x y] (binary x y))
  ([x y & more] nary))

;; ARITHMETIC

(def ^:const inf Double/PositiveInfinity)
(def ^:const nan Double/NaN)
(def ^:const eps Double/Epsilon)
(def ^:const finf Single/PositiveInfinity)
(def ^:const fnan Single/NaN)
(def ^:const feps Single/Epsilon)

(def ^:const operators
  (merge {:clojure.core/multiary '[+ - * / max min] :clojure.core/binary '[+ * max min mod quot] :clojure.core/unary '[- /]}
         {:System.Math/unary '[abs acos asin atan ceiling cos cosh exp floor log log10 round sign sin sinh sqrt tan tanh truncate]
          :System.Math/binary '[atan2 pow]}
       #_{:UnityEngine.Mathf/unary '[abs acos asin atan ceil cos exp floor log round sign sin sqrt tan]
          :UnityEngine.Mathf/binary '[atan2 pow]}))

(defmacro do-defmultis []
  `(do ~@(mapcat #(for [op (-> % val)] (list 'defmulti op (-> % key name symbol) :hierarchy #'*h*)) ;:hierarchy #'*h*))
                 operators)))

(do-defmultis)

;; Axioms

(defmethod + nil [] 0)
(defmethod + ::any [x] x)
(defmethod + [Boolean Boolean] [x y] (or x y))
(defmethod + [Complex Complex] [a b]
  (Complex. (+ (.x a) (.x b))
            (+ (.y a) (.y b))))

(defmethod - nil [] 0)
(defmethod - [::any ::any] [x y] (+ x (- y)))
(defmethod - Boolean [x] (not x))
(defmethod - [Boolean Boolean] [x y] 
  (and (boolean x) (not (boolean y))))
(defmethod - Complex [a] 
  (Complex. (- (.x a)) (- (.y a))))

(defmethod * nil [] 1)
(defmethod * ::any [x] x)
(defmethod * [Boolean Boolean] [x y] (and x y))
(defmethod * [Complex Complex] [a b]
  (Complex. (- (* (.x a) (.x b))
               (* (.y a) (.y b)))
            (+ (* (.x a) (.y b))
               (* (.y a) (.x b)))))

(defmethod / nil [] 1)
(defmethod / [::any ::any] [x y] (* x (/ y)))
(defmethod / Boolean [x] (not x))
(defmethod / [Boolean Boolean] [x y] 
  (and (not x) (boolean y)))
(defmethod / Complex [a]
  (let [den (+ (* (.x a) (.x a))
               (* (.y a) (.y a)))]
    (Complex. (/ (.x a) den)
              (/ (- (.y a)) den))))

;; COMPARISON

(defmethod max ::any [x] x)
(defmethod min ::any [x] x)

;; Element-wise ops

(defmacro defunary [op rfn]
 `(do (~'defmethod ~op ::cnumber ~(vary-meta ['x] assoc :no-magic true) (~rfn (double ~'x)))
      (~'defmethod ~op ::number ~'[x] (~rfn ~'x))
      (~'defmethod ~op ~'Sequential ~'[x] (map ~op ~'x))
      (~'defmethod ~op ~'APersistentVector ~'[x] (mapv ~op ~'x))
      (~'defmethod ~op ~'MapEntry ~'[x] (~op (val ~'x)))
      (~'defmethod ~op ~'IPersistentMap ~'[x]
        (loop [~'acc ~'x ~'in (keys ~'x)]
          (if-let [~'k (first ~'in)]
            (recur (update ~'acc ~'k ~op) (rest ~'in)))))))

(defmacro defbinary [op rfn]
 `(do (defmethod ~op ~[::cnumber ::cnumber] ~(vary-meta ['x 'y] assoc :no-magic true) (~rfn (double ~'x) (double ~'y)))
      (defmethod ~op ~[::number ::cnumber] ~(vary-meta ['x 'y] assoc :no-magic true) (~rfn ~'x (double ~'y)))
      (defmethod ~op ~[::cnumber ::number] ~(vary-meta ['x 'y] assoc :no-magic true) (~rfn (double ~'x) ~'y))
      (defmethod ~op ~[::cnumber 'Complex] ~'[x y] (~op (~'Complex. ~'x 0) ~'y))
      (defmethod ~op ~['Complex ::cnumber] ~'[x y] (~op ~'x (~'Complex. ~'y 0)))
      (defmethod ~op ~[::number ::number] ~'[x y] (~rfn ~'x ~'y))
      (defmethod ~op ~['Sequential ::number+] ~'[x y] (map ~op ~'x (repeat ~'y)))
      (defmethod ~op ~[::number+ 'Sequential] ~'[x y] (map ~op (repeat ~'x) ~'y))
      (defmethod ~op ~['APersistentVector ::number+] ~'[x y] (mapv ~op ~'x (repeat ~'y)))
      (defmethod ~op ~[::number+ 'APersistentVector] ~'[x y] (mapv ~op (repeat ~'x) ~'y))
      (defmethod ~op ~['IPersistentMap ::number+] ~'[x y] (~op ~'x (zipmap (keys ~'x) (repeat ~'y))))
      (defmethod ~op ~[::number+ 'IPersistentMap] ~'[x y] (with-meta (~op (zipmap (keys ~'y) (repeat ~'x)) ~'y) (meta ~'y)))
      (defmethod ~op ~'[Sequential Sequential] ~'[x y] (map ~op ~'x ~'y))
      (defmethod ~op ~'[Sequential APersistentVector] ~'[x y] (map ~op ~'x ~'y))
      (defmethod ~op ~'[APersistentVector Sequential] ~'[x y] (mapv ~op ~'x ~'y))
      (defmethod ~op ~'[APersistentVector APersistentVector] ~'[x y] (mapv ~op ~'x ~'y))
      (defmethod ~op ~'[MapEntry MapEntry] ~'[x y] (~op (val ~'x) (val ~'y)))
      (defmethod ~op ~['MapEntry ::any] ~'[x y] (~op (val ~'x) ~'y))
      (defmethod ~op ~[::any 'MapEntry] ~'[x y] (~op ~'x (val ~'y)))
      (defmethod ~op ~'[IPersistentMap IPersistentMap] ~'[x y] (merge-with ~op ~'x ~'y))
      (prefer-method ~op ~'[APersistentVector Sequential] ~'[Sequential APersistentVector])
      (prefer-method ~op ~'[Complex Complex] ~['Complex ::any])
      (prefer-method ~op ~'[Complex Complex] ~[::any 'Complex])
      (prefer-method ~op ~['MapEntry ::any] ~['APersistentVector ::number+])
      (prefer-method ~op ~[::any 'MapEntry] ~[::number+ 'APersistentVector])))

(defmacro defmultiary [op rfn]
  `(clojure.core/defmethod ~op ~nary
    ~'[xo yo & moreo]
    (loop ~'[x xo y yo more moreo]
      (if (seq ~'more) ;reduce
        (recur (~op ~'x ~'y) (first ~'more) (rest ~'more))
        (~op ~'x ~'y)))))

;todo: other methods such as Lerp (trinary)

(defmacro do-defarys []
  `(do ~@(mapcat #(for [op (-> % val) :let [host-sym (symbol (-> % key namespace) (cond-> (str op) (-> % key namespace symbol camel?) capitalize))]]
                    (list (->> % key name (str "def") symbol) op host-sym))
                 operators)))

(do-defarys) ; takes too long!

(defmulti dot binary :hierarchy #'*h*)

(defmethod dot [::any ::any] [x y]
  (reduce + (map * x y)))

(clojure.core/defn norm [x]
  (sqrt (dot x x)))

;;  UNITY

(defmethod / Vector3 [x] (Vector3. (/ (.-x x)) (/ (.-y x)) (/ (.-z x))))
(defmethod / Vector4 [x] (Vector4. (/ (.-x x)) (/ (.-y x)) (/ (.-z x)) (/ (.-w x))))
(defmethod / Matrix4x4 [x] (.-inverse x))
(defmethod * [Vector3 ::cnumber] ^:no-magic ^:com [x y] (Vector3/op_Multiply x (double y)))
(defmethod * [Vector4 ::cnumber] ^:no-magic ^:com [x y] (Vector4/op_Multiply x (double y)))
(defmethod * [Matrix4x4 ::cnumber] ^:no-magic ^:com [x y] (* x (double y)))
(defmethod * [Vector3 ::number] ^:com [x y] (Vector3/op_Multiply x y))
(defmethod * [Vector4 ::number] ^:com [x y] (Vector4/op_Multiply x y))
(defmethod * [Matrix4x4 ::number] ^:com [x y] 
  (loop [x x i 4]
    (if (zero? i) x
      (let [i (dec i)]
        (recur (put-row x i (* (row x i) y)) i)))))
(defmethod * [Vector3 Vector3] [x y] (Vector3/Scale x y))
(defmethod * [Vector4 Vector4] [x y] (Vector4/Scale x y))
(defmethod * [Vector3 Matrix4x4] [y x] (.MultiplyPoint x y))
(defmethod * [Vector4 Matrix4x4] [y x] (Matrix4x4/op_Multiply x y))
(defmethod * [Matrix4x4 Vector3] [x y] (.. x -transpose (MultiplyPoint y)))
(defmethod * [Matrix4x4 Vector4] [x y] (Matrix4x4/op_Multiply (.-transpose x) y))
(defmethod * [Matrix4x4 Matrix4x4] [x y] (Matrix4x4/op_Multiply x y))
(defmethod dot [Vector3 Vector3] [x y] (Vector3/Dot x y))
(defmethod dot [Vector4 Vector4] [x y] (Vector4/Dot x y))
(defmethod dot [Quaternion Quaternion] [x y] (Quaternion/Dot x y))

;; GEOMETRIC

(clojure.core/defn eps= 
  ([x y] (eps= x y eps))
  ([x y eps] (-> (- x y) abs (< eps))))

(def ^:const pi Math/PI)
(def ^:const tau (* pi 2))
(def ^:const pi:2 (/ pi 2))
(def ^:const pi:3 (/ pi 3))
(def ^:const pi:4 (/ pi 4))
(def ^:const pi:6 (/ pi 6))
(def ^:const g (norm Physics/gravity))

(clojure.core/defn dist [x y]
  (sqrt (apply + (pow (- x y) 2))))

(clojure.core/defn ll
  ([x] x)
  ([x y] (/ (* x y) (+ x y)))
  ([x y & more] (reduce ll (ll x y) more)))

;; UNITS

(def ^:const fps 30)
(def ^:const dt (/ fps))
(def ^:const rpm.s:rad (/ 60 tau))
(def ^:const m.h:km.s 3.6)
(def ^:const deg:rad (/ 180 pi))
(def ^:const m:in 0.0254)
(def ^:const m:rad.in (/ 0.0254 tau))

;; BOUNDARY

(clojure.core/defn constrain
  ([v val] (-> v (max (- val)) (min val)))
  ([v inf sup] (-> v (max inf) (min sup))))

(clojure.core/defn lmap [v o O p P]
  (->> (- (+ o eps) O) / (* (- v o) (- p P)) (+ p)))

(clojure.core/defn mcycle
  ([v val] (-> v (lmap (- val) val 0 1) (mod 1) (lmap 0 1 (- val) val)))
  ([v inf sup] (-> v (lmap inf sup 0 1) (mod 1) (lmap 0 1 inf sup))))

(clojure.core/defn bound [expr inf sup mode]
  (condp = mode
    :cycle (mcycle expr inf sup)
    :constrain (constrain expr inf sup)
    expr))

;; MAGIC

(comment "figure out which spells won't work"
  (require '[clojure.spec :as s]
           '[arcadia.core])

  (s/def ::arg-dsl
    (s/or :str (s/and symbol? #(ends-with? (name %) "-string?"))
          :map (s/and symbol? #(ends-with? (name %) "-map?"))
          :reg #{'+ '* '?}
          :and #{'&}
          :sym symbol?
          :any any?))

  (clojure.core/defn args->call [fn-sym args]
    (letfn [(rfn [[args* call :as acc] arg]
              (case (key (s/conform ::arg-dsl arg))
                :str `([~@args* ~arg] (~@call ""))
                :map `([~@args* ~arg] (~@call {}))
                :and `([~@args* ~arg] (clojure.core/apply ~@call))
                :any `([~@args* arg#] (~@call arg#))
                :reg acc
                `([~@args* ~arg] (~@call ~arg))))]
      (reduce rfn `([] (~fn-sym)) args)))

  (clojure.core/defn mock
    ([a-var]
     (-> (str a-var)
         (subs 2)
         (symbol)
         (mock (-> a-var meta :arglists))))
    ([sym arglists]
     (let [mock-sym (-> sym name (str "$") symbol)]
       `(~mock-sym ~@(map #(args->call sym %) arglists)))))

  (clojure.core/defn static-xf [from-ns]
    (comp (filter #(-> % val meta :static)) ;name and bitflip are not static, possibly due a typo (but they work)
          (remove (some-fn #(-> % key #{'to-array 'to-array-2d 'with-bindings* 'bound-fn* 'list*}) ;these won't even compile
                           #(-> % val meta :tag (= Boolean) (and (-> % key (not= 'nil?)))) ;these always return nil
                           #(-> % val meta :private)
                           #(-> % val meta :macro)))
          (map #(mock (->> % key str (symbol (str from-ns)))
                      (-> % val meta :arglists)))))

  (defmacro mock-static-fns [from-ns]
    (let [static-fns (sequence (static-xf from-ns) (ns-interns from-ns))]
      `(do ~@(map-indexed #(list 'do (cons 'magic.api/defn %2) `(arcadia.core/log ~%1) `(println ~%1))
                  static-fns))))

  (mock-static-fns clojure.core))