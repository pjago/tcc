(do (in-ns 'pjago.quad)
    (require '[common.repl :as re]
             '[clojure.core.logic :as lo]
             '[clojure.spec :as s]
             '[clojure.spec.test :as test]
             '[clojure.spec.gen :as gen]))

(defn ctor [sym]
  (if-let [utype (re/resolve-but-dont-break-please sym)]
    (as-> (str/split (str utype) #"\.") s
          (update s (dec (count s)) #(str "/->" %))
          (str (str/join "." (pop s)) (peek s))
          (if-let [try (resolve (symbol s))]
            #(apply (deref try) %)
            #(System.Activator/CreateInstance utype (to-array %))))))

(defmacro from-spec [utype from]
  `(s/with-gen #(instance? ~utype %)
               #(gen/fmap (ctor ~utype) (s/gen ~from))))

(def eps? #(m/eps= %1 0 1e-6))
(defn up? [x]
  (let [norm (m/norm x)]
    (if (zero? norm) 0
      (-> (m// x (m/norm x))
          (m/* (v3+ forward right))
          (m/norm)))))

(s/def ::number (s/float-in :min -1000 :max 1000))
(s/def ::v3-space (s/tuple ::number ::number ::number))
(s/def ::v3 (from-spec Vector3 ::v3-space))
(s/def ::up (s/with-gen (s/and ::v3 (comp eps? up?))
                       #(s/gen #{up})))

(s/def ::aero-space
  (s/tuple (s/double-in :min 8.0 :max 10.0) 
           (s/double-in :min 4.0 :max 6.0)
           ::v3))

(s/def ::uiuc (from-spec UIUC ::aero-space))
(s/def ::stables (from-spec Stables ::aero-space))
(s/def ::aero (s/or :uiuc ::uiuc :stables ::stables))

(defn drag [dynamics w v] (.Drag dynamics w v))
(defn thrust [^IAero dynamics w v] (.Thrust dynamics w v))

(s/fdef thrust
  :args (s/cat :dynamics ::aero :w ::v3 :v ::v3)
  :ret number?)

;; REPL

(test/instrument `thrust)
(mapv test/abbrev-result (test/check `thrust))