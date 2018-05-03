(in-ns common.repl)

;; IDENTITY

(defmacro def= [name equiv]
  `(let [equiv# ~equiv]
    (defn ~name 
      ([x#] true)
      ([x# y#] (equiv# x# y#))
      ([x# y# & more#]
       (if (equiv# x# y#)
         (if (next more#)
           (recur y# (first more#) (next more#))
           (equiv# y# (first more#)))
         false)))))

(def ^:dynamic *h* (make-hierarchy))

(def= o= obj-equiv)
(def= p= #(= (path %1) (path %2)))
(def= h= #(isa? *h* (keypath %1) (keypath %2)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; RENDER (OLD)

(require '[clojure.zip :as z])

(defn ensure-cloned
 ([id] (ensure-cloned id nil))
 ([id -utype]
  (let [utype (-ensure-type -utype)]
    (if-let [src (resource id -utype)]
      (if (or (not (instance? utype id)) (= src id))
        (clone! id -utype)
        id)))))

(defn til
  ([zipper fun]
   (->> (iterate fun (fun zipper))
        (eduction (take-while some?)
                  (drop-while z/branch?)
                  (take 1))
        (first)))
  ([zipper fun & funs]
   (->> (reverse (cons fun funs))
        (map #(fn [v] (if (some identity v) (% v))))
        (apply comp)
        (til zipper))))

(defn sequential-zip [scene]
  (z/zipper sequential? seq
    (fn [node children]
      (with-meta (vec children) (meta node)))
    scene))

(defn leaf [id obj]
  (if-let [id (if (satisfies? ISceneGraph id) id (object-search id GameObject))]
    (->> (gobj-seq id)
         (filter #(o= obj %))
         (first))))

(defn manager [scene]
  (fn [rfn]
    (let [zop (volatile! (sequential-zip scene))
          job (volatile! [])]
      (fn ([] (rfn))
          ([acc]
           (run! #(.SetActive % true) @job)
           (-> @zop z/root sequential-zip (til z/next)
               z/node rfn))
          ([acc id]
           (vswap! zop til z/next)
           (when-let [gob (ensure-cloned id GameObject)]
             (.SetActive gob false)
             (if-let [p (til @zop z/up z/up z/next)]
               (child+ (z/node p) gob))
             (vswap! zop z/replace gob)
             (vswap! job conj gob)
             (rfn acc gob)))))))

(defn render!
  ([scene] (transduce (manager scene) conj [] (flatten scene)))
  ([scene & scenes] (mapv render! (cons scene scenes))))

;; LOGIC

(do (use '[clojure.core.logic :exclude [log]])
    (require '[clojure.core.logic]
             '[clojure.core.logic.protocols :refer [walk]]))

(defn sourceo [src obj utype]
  (fn [a]
    (let [obj (walk a obj)
          src (walk a src)]
      (condp = [(not (lvar? obj))
                (not (lvar? src))]
        [true false]
        (->> (resources obj utype)
             (map #(unify a src %))
             (to-stream))
        [false true]
        (->> (objects-search src utype)
             (map #(unify a obj %))
             (to-stream))
        [true true]
        (when (and (instance? UnityEngine.Object obj)
                   (empty? (AssetDatabase/GetAssetPath obj))
                   (o= src obj))
          a)))))

(defn cmpto [obj cmp utype]
  (fn [a]
    (let [obj (walk a obj)
          cmp (walk a cmp)]
      (condp = [(not (lvar? obj))
                (not (lvar? cmp))]
        [true false]
        (->> (cmpts obj utype)
             (map #(unify a cmp %))
             (to-stream))
        [false true]
        (unify a obj (.-gameObject cmp))
        [true true]
        (when (contains? (set (cmpts obj utype)) cmp)
          a)))))

(defn roleo [obj pro]
  (fn [a]
    (let [obj (walk a obj)
          pro (walk a (if (map? pro) (partial-map pro) pro))]
      (if (not (lvar? obj))
        (if-cmpt obj [st ArcadiaState]
          (unify a pro (roles obj)))))))

(defmacro dorun* [bindings & body]
  `(doseq
     ~(into []
        (mapcat (fn [[q goal]] [q `(run* [~q] ~goal)]))
        (partition 2 bindings))
     ~@body))
      
;; SPEC

(require '[clojure.spec :as s])

(defmacro from-spec [utype from]
  `(s/with-gen #(instance? ~utype %)
               #(gen/fmap (ctor ~utype) (s/gen ~from))))

(defn abbrev-result [check-result]
  (let [rabbit (comp :smallest
                     :shrunk
                     :clojure.spec.test.check/ret)]
    ((juxt :sym rabbit :failure) check-result)))

;; TRASH

(defn vgob [gs]
  (let [ret (new |UnityEngine.GameObject[]| (count gs))
        vgs (vec gs)]
    (dotimes [i (count gs)]
      (aset ret i (nth vgs i)))
    ret))

(defn message-key [hook-type]
  (-> (re-find #".*(?=Hook)" (str hook-type))
      (str/replace #"([a-z])([A-Z])" "$1-$2")
      str/lower-case 
      keyword))

(defn pass-meta [x id]
  (if-not (instance? clojure.lang.IMeta x) x
    (cond-> x 
      (instance? clojure.lang.IMeta id)
      (vary-meta merge (meta id))
      (aka id)
      (vary-meta merge {:name (symbol (aka id)) 
                        :ns (where id)}))))