(ns pjago.watch
  (:use arcadia.core)
  (:import (UnityEngine GameObject))
  (:import (clojure.lang Counted
                         Sequential
                         IPersistentCollection
                         IPersistentStack
                         IPersistentMap
                         Indexed
                         Associative
                         Reversible
                         IObj)
           (System.IO TextWriter))
  (:require ; [amalloy.ring-buffer :refer [ring-buffer]]
            ; [common.repl :refer [path]]
            [clojure.string :as str]))

;; RING BUFFER due to a bug with metadata not found

(set! *unchecked-math* true)

;challenge here: maybe don't create a new RingBuffer.
;question: will everything break?
;naah. let's just declare the constructor

(declare ->RingBuffer)

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

(defn ring-buffer
  "Create an empty ring buffer with the specified [capacity]."
  [capacity]
  (->RingBuffer 0 0 (vec (repeat capacity nil)) nil))

;;;

(defn watch-path [k]
  (cond
    (keyword? k)
    (str/replace (subs (str k) 1) \. \/)
    (instance? UnityEngine.GameObject k)
    (str/replace (subs (.tag k) 1) \. \/)))

(defmacro select-props [obj keys]
  (let [gobj (gensym "obj")]
    `(let [~gobj ~obj]
       (array-map
        ~@(mapcat 
            (fn [k] [k (-> (str/split (subs (str k) 1) #"-")
                           (#(cons (first %) (mapv str/capitalize (rest %))))
                           (->> (apply str) symbol
                                (list '. gobj)))])
            keys)))))

(def watch-root (str "./data/" (.ToUnixTimeSeconds System.DateTimeOffset/UtcNow)))

(def ^{:private true} watch-db (atom {}))
  
(defprotocol IWatch
  (watch [obj])
  (check [obj]))

;todo: check-js and watch-csv fns, that call watch and check
;check-js should return a string or a map 'almost' js
(defn check-js [obj]
  (letfn [(step [v]
            (cond
             (map? v)
             (check-js v)
             (or (instance? clojure.lang.LazySeq v) 
                 (instance? clojure.lang.Repeat v)
                 (instance? clojure.lang.Cycle v))
             (str (type v))
             (sequential? v)
             (mapv step v)
             (instance? arcadia.core.IMutable v)
             [(str (type v)) (hash v)]
             (nil? v) 'null
             (and (number? v) (or (Double/IsInfinity v) (Double/IsNaN v)))
             (str v) ;this is wrong
             (enum? v)
             (int v)
             (instance? Type v)
             (str v)
             :else v))]
    (into {}
      (map (fn [[k v]]
             [(if (keyword? k)
                (keyword (subs (str/replace (str k) #"[\-|\/]" "_") 1))
                k)
              (step v)]))
      (check obj))))

(extend-protocol IWatch
  nil
  (check [_] nil)
  (watch [_] nil)
  clojure.lang.APersistentMap
  (check [obj] obj)
  (watch [obj] obj)
  clojure.lang.APersistentVector
  (check [obj]
    {:vals (mapv (fn [o] (merge (check-js o) {:type (str (type o)) :hash (hash o)})) 
                 obj)})
  (watch [obj]
    (if-let [w (seq (keep watch obj))]
      (reduce #(update %1 (key %2) (fnil conj []) (val %2)) {} 
        (mapcat seq w)))))

;serialize here to reduce performance hit on spit-watch @todo
(defn watch-state [^UnityEngine.GameObject gob k]
  (let [watch-map (into {}
                    (keep #(if-let [w (watch (state gob %))] [% w]))
                    (state gob k))]
    (if-not (empty? watch-map)
      (swap! watch-db update gob (partial merge-with conj) watch-map))))

(defn watch+
  ([^GameObject gob k] (watch+ gob k 1001))
  ([^GameObject gob k size]
   (if-let [obj (state gob k)]
     (let [buf (with-meta (ring-buffer size) (check-js obj))]
       (swap! watch-db assoc-in [gob k] buf)
       (if (pos? size)
         (hook+ gob :fixed-update ::watch #'watch-state))
       (update-state gob ::watch (fnil conj #{}) k))
     (log (str "Failed to add watch on " (.tag gob) " " k)))
   gob))

(defn watch- [^GameObject gob k]
  (update-state gob ::watch disj k)
  (swap! watch-db update gob dissoc k)
  gob)

(defn clear-watch 
  ([] 
   (run! clear-watch (keep obj-nil (keys @watch-db))))
  ([^GameObject gob]
   (hook- gob :fixed-update ::watch)
   (state- gob ::watch)
   (swap! watch-db dissoc gob)
   gob))

(defn spit-watch
  ([^GameObject gob] (spit-watch gob {}))
  ([^GameObject gob info]
   (let [folder (str watch-root "/" (watch-path gob))
         wuid (gensym "")
         info (check-js (merge info {:wuid (int (str wuid))}))
         watch-keys (state gob ::watch)
         watch-vals (select-keys (state gob) watch-keys)
         gob-db (get @watch-db gob)]
     (letfn [(spit-key [k]
               (let [obj (get watch-vals k)
                     file (str folder "/" (watch-path k) "/" wuid)
                     buf (get gob-db k)
                     chk (meta buf)                                   ;#"(?<= ).*" ;this is slowwwww
                     vf (fn [x] (str/join "," (map (comp #(str/trim (or (re-find #"[(?>=\[)|(?<= )].*" %) %)) print-str) (vals x))))
                     kf (fn [x] (str/join "," (map #(str/replace (name %) "-" "_") (keys x))))
                     ; cf (fn [[k v]] [k (cond (enum? v) (int v) (instance? Type v) (str v)
                     ;                         (and (number? v) (or (Double/IsInfinity v) (Double/IsNaN v))) ;haha
                     ;                         (str v)
                     ;                         ; (sequential? v) ;this would be too many layers of str @bug
                     ;                         ; (str \[ (str/join "," v) \])
                     ;                         :else v)])
                     js (merge info chk (check-js {:type (type obj) :hash (hash obj)}))]
                 (System.IO.Directory/CreateDirectory (re-find #".*(?=/)" file))
                 (if-not (empty? buf)
                   (spit (str file ".csv") (str/join "\n" (cons (kf (nth buf -1)) (sequence (map vf) buf)))))
                 (spit (str file ".json") ;todo: use an actual edn to json converter
                       (-> (str js)
                           (str/replace #":([^\s]+) " "\"$1\": ")
                           (str/replace #"(?<=[-.0-9]) (?=[-.0-9])" ",")
                           (str/replace #"(?<=\}) (?=\{)" ",")
                           (str/replace #"(\[[^\s]*\") " "$1,")
                           (str/replace #"#[^\s]* " "")))
                 file))]
       (if (empty? gob-db)
         (future nil)
         (do (log (str "Saving at: " folder " (" wuid ")"))
             (future (mapv spit-key watch-keys))))))))