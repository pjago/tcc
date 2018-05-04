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

; (set! *unchecked-math* true)

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

(def watch-root (str "../../TCC/data/" (.ToUnixTimeSeconds System.DateTimeOffset/UtcNow)))

(def ^{:private true} watch-db (atom {}))
  
(defprotocol IWatch
  (watch [obj])
  (check [obj]))

(extend-protocol IWatch
  nil
  (check [_] nil)
  (watch [_] nil))

;todo: check-js and watch-csv fns, that call watch and check
;check-js should return a string or a map 'almost' js
(defn check-js [obj]
  (into {}
    (map (fn [[k v]] [k (if (instance? arcadia.core.IMutable v)
                          [(str (type v)) (hash v)] 
                          v)]))
    (check obj)))

(defn watch-state [^UnityEngine.GameObject gob k]
  (swap! watch-db update gob (partial merge-with conj)
    (into {}
      (map (fn [v] [v (watch (state gob v))]))
      (state gob k))))

(defn watch+
  ([^GameObject gob k] (watch+ gob k 1001))
  ([^GameObject gob k size]
   (if-let [obj (state gob k)]
     (let [buf (with-meta (ring-buffer size) (check-js obj))]
       (swap! watch-db assoc-in [gob k] buf)
       (hook+ gob :fixed-update ::watch #'watch-state)
       (update-state gob ::watch (fnil conj #{}) k)))
   gob))

(defn watch- [^GameObject gob k]
  (update-state gob ::watch disj k)
  (swap! watch-db update gob dissoc k)
  gob)

(defn clear-watch 
  ([] (run! clear-watch (keys @watch-db)))
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
         info (merge info {:wuid (int (str wuid))})
         watch-keys (state gob ::watch)
         watch-vals (select-keys (state gob) watch-keys)
         gob-db (get @watch-db gob)]
     (letfn [(spit-key [k]
               (let [obj (get watch-vals k)
                     file (str folder "/" (watch-path k) "/" wuid)
                     buf (get gob-db k)
                     chk (meta buf)
                     wch (nth buf -1)
                     vf (fn [x] (str/join "," (map (comp #(or (re-find #"(?<= ).*" %) %) print-str) (vals x))))
                     kf (fn [x] (str/join "," (map #(str/replace (name %) "-" "_") (keys x))))
                     cf (fn [[k v]] [k (cond (enum? v) (int v) (instance? Type v) (str v) :else v)])
                     js (merge chk {:type (type obj) :hash (hash obj)} info)]
                 (System.IO.Directory/CreateDirectory (re-find #".*(?=/)" file))
                 (spit (str file ".csv") (str/join "\n" (cons (kf wch) (sequence (map vf) buf))))
                 (spit (str file ".json") ;todo: use an actual edn to json converter
                       (-> (str (into {} (map cf) js))
                           (str/replace #":([^\s]+) " "\"$1\": ")
                           (str/replace #"(?<=[-.0-9]) (?=[-.0-9])" ",")
                           (str/replace #"(\[[^\s]*\") " "$1,")
                           (str/replace #"#[^\s]* " "")))
                 file))]
       (future (mapv spit-key watch-keys))))))