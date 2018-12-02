(ns common.processing
  (:use [arcadia.core :exclude [if-cmpt log children]] arcadia.linear)
  (:import [UnityEngine Application Space Rigidbody Quaternion Transform GameObject MeshRenderer Rigidbody RigidbodyConstraints]
           [clojure.lang IMeta IDeref]
           ArcadiaState ArcadiaBehaviour |ArcadiaBehaviour+IFnInfo[]|)
  (:refer-clojure :exclude [trampoline map replace tree-seq doto memoize])
  (:require [clojure.core :as c]
            [arcadia.core :as ac]
            [clojure.walk :as w]))

(declare tag)

(defn keypath [x]
  (cond 
    (keyword? x) x
    (instance? IMeta x)
    (as-> (meta x) m
          (keyword (str (:ns m))
                   (str (:name m))))
    (instance? GameObject x)
    (tag x)))

(defn resource [x ^System.Type t]
  (UnityEngine.Resources/Load
    (if-let [kx (keypath x)]
      (->> (subs (str kx) 1)
           (c/replace {\. \/})
           (apply str)))
    t))

(defn clone! [x]
  (c/doto (instantiate x)
    (-> (.name) 
        (set! (.name x)))))

;;TEMP? arcadia-unity #273

(def gentagged @#'ac/gentagged)
(def meta-tag @#'ac/meta-tag)

(defmacro if-cmpt
  "Execute body of code if `gob` has a component of type `cmpt-type`"
  [gob [cmpt-name cmpt-type] then & else]
  (let [gobsym (gentagged "gob__" 'UnityEngine.GameObject)]
    `(let [obj# ~gob]
       (if (obj-nil obj#)
         (with-gobj [~gobsym obj#]
           (if-let [~(meta-tag cmpt-name cmpt-type) (cmpt ~gobsym ~cmpt-type)]
             ~then
             ~@else))
         ~@else))))

;; COMP

(declare scene)

(defmacro fx [& body]
  `(fn ~'[& _] (do ~@body)))

(defn branch? [x]
  (or (fn? x) (var? x)))

(defn init [f1 f]
  (fn
    ([] (f1 (f)))
    ([acc in] (f acc in))
    ([acc] (f acc))))
    
(defn step [f2 f]
  (fn
    ([] (f))
    ([acc in] (f2 acc in) (f acc in))
    ([acc] (f acc))))

(defn complete [f1 f]
  (fn
    ([] (f))
    ([acc in] (f acc in))
    ([acc] (f1 acc) (f acc))))

(defn has-arity? [f n]
  (if (var? f)
    (.HasArity @f n)
    (.HasArity f n)))

(defn doto [f & fs]
  (let [f0 (rseq (filterv #(has-arity? % 0) fs))
        f1 (cons f (filter #(has-arity? % 1) fs))
        f2 (cons f (filter #(has-arity? % 2) fs))]
    (fn
      ([] (doseq [f f0] (f)) (f))
      ([acc in] (doseq [f f2] (f acc in)) acc)
      ([acc] (doseq [f f1] (f acc)) acc))))

;I should move map logic to init, after fab @todo
;but unlike fab, it has to precede flatten-wrap @bug
;the goal is to move fresh-state to init
(defn wrap 
  ([x] 
   (wrap x (completing (fn [acc in] acc))))
  ([x rf]
   (cond 
     (branch? x) x
     (some? x)
     (let [m (meta x)
           init (if (map? x) (::init m x) x)
           rf (fn
                ([] [init])
                ([acc in] (rf acc in))
                ([acc] (rf acc)))]
       (if (map? x)
         (as-> (complete #(roles+ % x) rf) rf
               (if-let [xf (::renderer m)]
                 ((xf) rf)
                 rf))
         rf)))))

;; last changes made it slow. todo: make everything faster
;; trampoline kind makes it obvious the init arity should have no side-effects @bug?
(defn trampoline
  ([x]
   (as-> (wrap x) x (trampoline (x) x)))
  ([x rf]
   (cond
     (branch? x)
     (let [root (trampoline (x) x)]
       (fn
         ([] (root))
         ([acc] (root acc) (rf acc))
         ([acc in] (root acc in) (rf acc in))))
     (sequential? x)
     (let [node (first x)
           root (trampoline node rf)]
       (fn
         ([] (concat (root) (rest x)))
         ([acc] (root acc))
         ([acc in] (root acc in))))
     (nil? x)
     (wrap (scene) rf)
     :else 
     (wrap x rf))))

;; TRAVERSAL

(defn map "maps xf on init" [xf] ;lazy
  (->> #(if (branch? %) (xf %) %)
       (partial c/map)
       (partial init)))

(defn tree-seq "tree-seqs xf on init" [xf] ;lazy
  (fn continue [rfn]
    (if-not (branch? rfn) rfn
      (init (partial c/map continue)
            (xf rfn)))))

(defn prewalk "prewalks f on init" [f] ;not lazy
  (fn continue [rfn]
    (if-not (branch? rfn) rfn
      (init (partial w/prewalk (comp continue f))
            rfn))))

(defn postwalk "postwalks f on init" [f] ;not lazy
  (fn continue [rfn]
    (if-not (branch? rfn) rfn
      (init (partial w/postwalk (comp f continue))
            rfn))))

;; PROTOCOLS

(defmutable SceneGraph 
  [children])

(defprotocol IResourceLocation
  (-tag ^clojure.lang.Keyword [this]))

(extend-protocol ISceneGraph
  SceneGraph
  (gobj [this] this)
  (parent [this] this)
  (children [this]
    (vec (.-children this)))
  (child+
    ([this child transform-to]
     (child+ this child))
    ([this child]
     (set! (.-children this) (conj (.-children this) child))
     nil))
  (child-
    ([this child transform-from]
     (child- this child))
    ([this child]
     (set! (.-children this) (disj (.-children this) child))
     nil)))

(extend-protocol IResourceLocation
  SceneGraph
  (-tag [scene] ::scene)
  GameObject
  (-tag [gob]
    (let [t (.tag gob)]
      (if (= (nth t 0) \:)
        (keyword (subs t 1))
        (keyword "unity" t)))))

(def tag 
  (c/memoize -tag))

(defn by-tag 
  ([coll] (group-by tag coll))
  ([coll fun]
   (reduce-kv #(assoc %1 %2 (fun %3)) {} (group-by tag coll))))

(def s> 
  #(object-tagged (str %)))

(def ss>
  #(objects-tagged (str %)))

(defn scene [] 
  (->SceneGraph #{}))

;; UTIL

(defn freeze [pos & [rot]]
  (letfn [(fpos [x]
            (case x
              :x RigidbodyConstraints/FreezePositionX
              :y RigidbodyConstraints/FreezePositionY
              :z RigidbodyConstraints/FreezePositionZ))
          (frot [x]
            (case x
              :x RigidbodyConstraints/FreezeRotationX
              :y RigidbodyConstraints/FreezeRotationY
              :z RigidbodyConstraints/FreezeRotationZ))]
    (let [c (transduce (c/map frot) enum-or
                (transduce (c/map fpos) enum-or 
                  RigidbodyConstraints/None
                  pos)
                rot)]
      (partial complete
        #(with-cmpt % [rb Rigidbody]
           (set! (.constraints rb) 
                 (enum-or (.constraints rb) c)))))))

(defn unfreeze [pos & [rot]]
  (letfn [(fpos [x]
            (case x
              :x RigidbodyConstraints/FreezePositionX
              :y RigidbodyConstraints/FreezePositionY
              :z RigidbodyConstraints/FreezePositionZ))
          (frot [x]
            (case x
              :x RigidbodyConstraints/FreezeRotationX
              :y RigidbodyConstraints/FreezeRotationY
              :z RigidbodyConstraints/FreezeRotationZ))]
    (let [c (transduce (c/map frot) enum-or
                (transduce (c/map fpos) enum-or 
                  RigidbodyConstraints/None
                  pos)
                rot)
          c (bit-and (int RigidbodyConstraints/FreezeAll)
                     (bit-not (int c)))]
      (partial complete
        #(with-cmpt % [rb Rigidbody]
           (set! (.constraints rb) 
                 (enum-and (.constraints rb) c)))))))

(defn paint 
  ([material] (paint material false))
  ([material gob-seq?]
   (partial complete
     (as-> #(if-cmpt % [mr MeshRenderer]
              (set! (.-sharedMaterial mr) material)) p
           #(if (satisfies? IEntityComponent %) (p %))
           (if gob-seq?
             #(doseq [g (gobj-seq %)] (p g))
             p)))))

(defn memoize [rf]
  (let [done (volatile! nil)]
    (fn 
      ([] (if-let [x @done] x (vreset! done (rf))))
      ([gob child] (rf gob child))
      ([gob] (rf gob)))))

(defn log [f]
  #(fn 
     ([]
      (ac/log "pre\tinit\t\t" %)
      (let [x (%)] 
        (ac/log "post\tinit\t\t" (f (first x)))
        x))
     ([gob]
      (ac/log "pre\tcomplete\t" (f gob))
      (let [x (% gob)]
        (ac/log "post\tcomplete\t" (f x))
        x))
     ([gob child]
      (ac/log "pre\tstep\t\t" (f gob))
      (let [x (% gob child)]
        (ac/log "post\tstep\t\t" (f x))
        x))))

;fresh addresses the same problem as defmutable-once,
;but through walking a roundtrip of snapshot and mutable
;maybe this should after the db init?? @potential bug?
;and then wrap for maps would have add roles before that (at fab)
(defn fresh-state [rf]
  (let [mm @#'ac/maybe-mutable
        ms @#'ac/maybe-snapshot
        m (c/memoize #(w/walk (comp (partial %1 %1) ms) mm %2))]
    (doto rf
      #(if-cmpt % [as ArcadiaState]
         (reduce-kv state+ %
           (m m (state %)))))))
    ; (doto rf ;complete ;init
    ;   (fn [h] ;[h :as b]]
    ;     (if-cmpt h [as ArcadiaState]
    ;       (reduce-kv state+ h (m m (state h))))))))
    ;     ;b)
    ;   ;rf)))

;; todo: option where it mantains the activeSelf of resource
(defn set-active [self]
  (partial complete
    (fn [gob]
      (if (instance? GameObject gob)
        (.SetActive gob self)))))

(defn children [transform-to]
  (partial step
    (fn [gob child]
      (if (satisfies? ISceneGraph gob)
        (child+ gob child transform-to)))))

(defn replace [lookup]
  (partial init
    #(let [node (first %)]
       (cons (lookup node) (rest %)))))

(defn with-db [lookup]
  (partial init
    (partial w/postwalk
      #(if (branch? %) % (lookup %)))))

(defn with-xf [renderer-map]
  (replace
    (fn [x]
      (reduce-kv #(if (isa? x %2) (%3 %1) %1)
                 (wrap x)
                 renderer-map))))

(def with-state
  (partial init
    (fn [[h :as b]]
      (if (satisfies? IEntityComponent h)
        (ensure-cmpt h ArcadiaState))
      b)))

(def initialize-state
  (partial init
    (fn [[h :as b]]
      (if-cmpt h [as ArcadiaState]
        (do (set! (.fullyInitialized as) false)
            (.Initialize as)))
      b)))

;hooks are normaly executed in the order they're added.
;with execution-order you can specify the order by last
(defn execution-order [message-kw-groups]
  (letfn [(compare-by [ks]
            (comp (if (vector? ks) 
                    (zipmap ks (range (count ks)))
                    ks)
                  #(.key %)))]
    #(doto %
       (fn [gob]
         (doseq [[mk group] message-kw-groups
                 :let [k (compare-by group)]
                 h (cmpts gob (hook-types mk))
                 :let [ifns (aclone (.ifnInfos h))]]
           (.RemoveAllFunctions h)
           (.InvalidateIndexes h)
           (doseq [f (sort-by k ifns)]
             (.AddFunction h (.fn f) (.key f))))))))

;todo: 1-arity for hierarchy insertion
(defn transform-to [position rotation]
  (partial step
    (fn [gob child]
      (let [tr (.transform child)]
        (set! (.. child transform localPosition)
              (v3+ (.localPosition tr) position))
        (set! (.. child transform localRotation)
              (qq* (.localRotation tr) rotation))))))

;; RENDER

(def ^{:doc "flattens init to be [node & branches]"}
  flatten-wrap
  (comp
    (partial init
      #(let [[h & t] (flatten %)]
         (cons h (keep wrap t))))
    trampoline))
    
(defn unwrap "lazy-seq of branches interleaved with their init"
  [root]
  (lazy-seq
    (when (branch? root)
      (let [root (trampoline root)
            branch (root)]
        (->> (mapcat unwrap branch)
             (cons branch)
             (cons root))))))

(defn fab "clones from id's resource, if found.\n  attempts to tag by id."
  [id]
  (if-let [src (resource id GameObject)]
    (let [old (.activeSelf src)]
      (.SetActive src false)
      (let [gob (clone! src)]
        (.SetActive src old)
        (try
          (set! (.tag gob) (str (keypath id)))
          (catch Exception e
            (str "Try " (list 'tag-all! id))))
        gob))
    id))

(def ^{:doc "default value of *renderer*"}
  renderer
  (tree-seq
    (comp
      (children false)
      (set-active true) ;maybe I don't need this here?
      fresh-state
      initialize-state
      with-state
      (replace fab)
      flatten-wrap)))

(def ^{:dynamic true :doc "xf applied by render.\n  can be restored with (restore-renderer)"}
  *renderer* renderer)

(defn restore-renderer "restores *renderer* var-root" []
  (alter-var-root #'*renderer* (fn [_] @#'renderer)))

(defn renders "xf interface of render, useful for mocking an init"
  ([x node tail] (init (fx (cons node tail)) (wrap x)))
  ([x tail] (init #(concat % tail) (wrap x)))
  ([x] (wrap x)))

(def ^{:doc "transduces from root recursively"}
  render
  (comp
    (fn [root]
      (let [v (volatile! [])
            f (*renderer* root)]
        (letfn [(-render [f]
                  (let [[node & tail] (f)] ;;init
                    (vswap! v conj node)
                    (doseq [c (c/map -render tail)]
                      (f node c)) ;; step
                    (f node) ;;complete
                    node))]
          (-render f)
          @v)))
    renders))

(def ^{:doc "transduces from root in depth-first order"}
  batch-render
  (comp
    (fn [root]
      ;; init
      (let [job (->> (*renderer* root)
                     (unwrap)
                     (apply array-map))]
        ;; step
        (doseq [[rf [node & tail]] job
                child (keep #(first (job %)) tail)]
          (rf node child))
        ;; complete
        (mapv (fn [[rf [node _]]] (rf node)) job)))
    renders))

(defn gen-factory []
  (let [stack (volatile! [])]
    (fn [[rf [node & tail] :as in]]
      (let [[rf [top _ & tail] :as in] (peek @stack)]
        (when (some? in)
          (rf top node)
          (vswap! stack pop)
          (if (seq tail)
            (vswap! stack conj [rf (cons top tail)])
            (rf top))))
      (if (seq tail)
        (vswap! stack conj in)
        (rf node))
      node)))

(def ^{:doc "lazy-seq of rendering nodes"}
  lazy-render ;bug: couldn't use sequence (it was eager)
  (comp
    (fn [root]
      (->> (*renderer* root)
           (unwrap)
           (partition 2)
           (c/map (gen-factory))))
    renders))

(def ^{:doc "xf that groups a render by transform root"}
  root ;bug: (sequence root (lazy-render ...)) is eager
  (comp (drop-while nil?)
        (partition-by #(.. % transform root))
        (map first)))

;; DEBUG

; (defn unwrap-all "like unwrap, but eagerly applies f to all branches"
;   [f root]
;   (w/postwalk #(if (branch? %) (f %) %)
;               (unwrap root)))

;; NEW

; (defn initial-count [f]
;   (->> ((tree-seq flatten-wrap) (wrap f))
;        (unwrap-all #(dec (count (%))))
;        (take-nth 2)
;        (reduce +)
;        (inc)))

; (defn scene-count [f]
;   (->> ((tree-seq flatten-wrap) (wrap f))
;        (unwrap-all #(instance? SceneGraph (first (%))))
;        (partition 2)
;        (filter first)
;        (mapcat second)
;        (filter false?)
;        count))

;; broken?
; (defn pool "unwraps all f, lazy rendering the result, partioning if scene.\n  repeats n times."
;   ([f] (pool nil f)) ;todo: rewrite so that passing nil is not ok
;   ([n f]
;    (let [f ((tree-seq memoize) (wrap f))
;          s (scene-count f)
;          p (->> (repeat n f)
;                 (if (nil? n) (repeat f))
;                 (cons nil)
;                 (lazy-render)
;                 (drop 1)
;                 (filter ;todo: maybe dont use transform root
;                   #(if (instance? GameObject %)
;                      (with-cmpt % [tr Transform]
;                        (= tr (.root tr))))))]
;      (if-not (zero? s)
;        (partition s p)
;        p))))

(do 1)