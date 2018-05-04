(do (in-ns 'pjago.quad)
    (use 'pjago.watch)
    (require '[common.processing :as x :refer [render]]))

;; TCC

; deslizamento zero na interface com sólido (poeira no fan)
; teorema(?) do eixo intermediário (rotações instáveis)
; relação entre thrust, drag e a velocidade angular.
; como corrigir rotação e posição simultaneamente? (goto)
; aumento na estabilidade abaixando-se o centro de gravidade

;; WATCH

(extend-protocol IWatch
  ICtrl
  (check [obj]
    (->> obj snapshot :arcadia.core/dictionary 
         (merge {:dt Time/fixedDeltaTime})))
  (watch [obj]
    {:r (nth (.r obj) -1)
     :y (nth (.y obj) -1)
     :e (nth (.e obj) -1)
     :u (nth (.u obj) -1)})
  Rigidbody
  (check [obj]
    (select-props obj
      [:drag :angular-drag :constraints
       :mass :center-of-mass :inertia-tensor]))
  (watch [obj]
    (select-props obj
      [:position :rotation :velocity :angular-velocity])))

;; ROLES
(def quad-ex
  {:fixed-update
   (merge #:ctrl {:spin 10 :height 20 :euler 30}
          {:kinematic 100 :dynamic 200 :physx 300})})

(def quad-r
  {:offset {:state [0.0 0.0 0.0 0.0]}
   :physx {:fixed-update #'quad-physx}
   :ctrl/spin {:fixed-update #'spin-ctrl}})
 
(def propeller-r
  {:snap/connect {:on-transform-parent-changed #'snap-joint}
   :snap/break {:on-joint-break #'snap-joint}
   :spin/set {:state 0.0}
   :spin {:state 0.0 :fixed-update #'kinematic}
   :speed {:state 0.0 :fixed-update #'kinematic}
   :drag {:state 0.0 :fixed-update #'dynamic}
   :thrust {:state 0.0 :fixed-update #'dynamic}})

(def euler-pid
  {:x [map->PI_DCtrl]
   :y [map->PID2Ctrl :b 0.7 :c 0.5]
   :z [map->PI_DCtrl]
   :e [map->EulerCtrl]})
   
(defn new-ctrl []
  (let [x (apply new-pid 0.5 1.0 0.5 1 (euler-pid :x))
        y (apply new-pid 0.5 30 0.5 1 (euler-pid :y))
        z (apply new-pid 0.5 1.0 0.5 1 (euler-pid :z))]
    #:ctrl
    {:pitch {:state x}
     :yaw {:state y}
     :roll {:state z}
     :euler {:state (apply new-euler x y z (euler-pid :e))
             :fixed-update #'euler-ctrl}}))

;; TRANSDUCERS

(defn propeller
  ([] [::propeller])
  ([gob child])
  ([gob]
   (snap-joint gob :snap/connect)
   (roles+ gob propeller-r)
   (state+ gob :aero (->Stables 10.0 4.5 true))))

(defn arm
  ([gob child])
  ([] [::arm #'propeller ::motor ::forearm ::elbow ::hand])
  ([gob] (hook+ gob :on-transform-children-changed :loose #'loose-props)))

(defn quad
  ([] [::quad (repeat 4 #'arm) ::base ::lshell ::ushell])
  ([gob child]
   (if-cmpt child [crb Rigidbody]
     (if-not (.isKinematic crb)
       (as/let [(as/with-cmpt rb Rigidbody) gob
                (as/with-cmpt fj FixedJoint) child]
         (hook+ child :on-joint-break :snap/break #'snap-joint)
         (set! (.-connectedBody fj) rb)))))
  ([gob]
   (let [{:keys [::arm ::propeller]}
         (group-by x/tag (gobj-seq gob))]
     (if-not (state gob :ref) (state+ gob :ref gob))
     (if (second arm) (mirror! arm (v3 0) up))
     (dotimes [i (count propeller)]
       (-> (nth propeller i)
           (state :aero)
           (.-clockwise)
           (set! (zero? (mod i 2)))))
     (roles+ gob (merge quad-r (new-ctrl)))
     (state+ gob :props propeller))))

(defn physx+ [ks]
  #(x/doto %
     (fn [gob]
       (if-cmpt gob [rb Rigidbody]
         (when-not (.isKinematic rb)
           (doseq [k ks]
             (hook+ gob :fixed-update k #'missing-physx))))
       gob)))

(defn paint [materials]
  (let [m (cycle (map #(x/resource % Material) materials))
        i (volatile! 0)]
    (x/postwalk
      #(case %
         ::arm
         (do (vswap! i inc) %)
         (::elbow ::hand ::forearm)
         ((x/paint (nth m @i)) (x/wrap %))
         %))))

(defn sansbox
  ([] [:sansbox])
  ([gob child])
  ([gob]
   (if-cmpt (object-tagged ":camera-main") [cam Camera]
     (mario-cam! (.-gameObject cam) gob))))

(defn background
  ([] 
   [nil ::manager :light-main :other-light :camera-main :camera-top :target])
  ([_])
  ([_ child]))

;; VERSIONING

(defn quad-0 "camera follow, aero & ctrl info"
  ([] [::quad ::lever])
  ([gob child]
   (if (isa? (x/tag child) ::lever)
     (state+ gob :ref child))
   (if (and Application/isPlaying
            (isa? (x/tag child) ::base))
     (destroy child 3)))
  ([gob]
   (sansbox gob)
   (doseq [p (state gob :props)]
     (hook+ p :on-gui :aero #'aero-info)
     (set! (.enabled (cmpt p (hook-types :on-gui))) false))
   (if-cmpt (first (state gob :props)) [on (hook-types :on-gui)]
     (set! (.enabled on) true))
   (hook+ gob :on-gui :ctrl #'ctrl-info)))

(defn quad-1 "centralize kinematics and dynamics" 
  ([] [::quad ::batt])
  ([gob child])
  ([gob]
   (hook+ gob :fixed-update :kinematic #'kinematico)
   (hook+ gob :fixed-update :dynamic #'dynamico)
   (doseq [p (state gob :props)]
     (destroy-immediate (cmpt p Joint))
     (destroy-immediate (cmpt p Rigidbody))
     (clear-hook p :fixed-update))
   (log (state gob :ref))))

(defmacro defq [name n materials opt]
  (let [vars (mapv #(resolve (symbol (str "quad-" %))) n)]
    `(def ~name
       (-> (x/doto #'quad ~@vars #(roles+ % ~opt))
           (x/renders (mapcat #(rest (%)) ~vars))
           ((paint ~materials))
           ((x/execution-order ~quad-ex))))))

(defq qon [0 1] [:plastic-white :plastic-red]
  {:offset {:state [589 589 589 589]}
   :ctrl/euler nil})

(defq qeuler [0 1] [:plastic-white :plastic-red]
  {:offset {:state [600 600 600 600]}})

(defq qheight [0 1] [:plastic-white :plastic-red]
  {:ctrl/height {:state (new-pid 100 1.0 1.0 5)
                 :fixed-update #'height-ctrl}
   :ctrl/euler nil})

(defq qboth [0 1] [:plastic-white :plastic-red]
  {:ctrl/height {:state (new-pid 50 1.0 1.0 5)
                 :fixed-update #'height-ctrl}})

(defq qlook [0 1] [:plastic-white :plastic-red]
  {:ctrl/height {:state (new-pid 200 1.0 1.0 1)
                 :fixed-update #'height-ctrl}
   :plane/look {:state (new-pid 3.0 100 5.0 5)
                :fixed-update #'plane-ctrl}})

(defq qomni [0 1] [:plastic-white :plastic-red]
  {:ctrl/height {:state (new-pid 200 1.0 1.0 1)
                 :fixed-update #'height-ctrl}
   :plane/omni {:state (new-pid 3.0 100 5.0 5)
                :fixed-update #'plane-ctrl}})

(defq qgoto [0 1] [:plastic-white :plastic-red]
  {:ctrl/height {:state (new-pid 200 1.0 1.0 1)
                 :fixed-update #'height-ctrl}
   :plane/goto {:state (new-pid 3.0 100 5.0 5)
                :fixed-update #'plane-ctrl}})

;; TOOLS

(defn cycle-gui [^GameObject gob]
  (let [ps (state gob :props)
        on #(cmpt % (hook-types :on-gui))
        pe (some #(if (.enabled (on %)) %) ps)]
    (run! #(set! (.enabled (on %)) false) ps)
    (loop [ps (cycle ps)]
      (if (or (nil? pe) (= (first ps) pe))
        (doto (second ps) (-> on .enabled (set! true)))
        (recur (rest ps))))))

(defn full-watch [gob]
  (state+ gob :rigidbody (cmpt gob Rigidbody))
  (reduce-kv watch+ gob
    #:ctrl
    {:_/rigidbody 1001
     :euler 1001
     :yaw 1001
     :pitch 1001
     :roll 1001
     :height 1001
     :plane 1001}))
  
;; RENDER

(def simple-prop
  (x/renders
    (x/doto #'quad #'quad-1
      #(state+ % :offset [929])
      #(hook- % :fixed-update :ctrl/euler))
    :sansbox
    [(x/renders #'propeller ::qp/fixed nil)]))

(def armed-prop
  (x/renders
    (x/doto #'quad quad-1
      #(state+ % :offset [929])
      #(hook- % :fixed-update :ctrl/euler))
    :sansbox
    [#'arm]))

;; next level

(defmutable RelayCtrl
  [n ;period in deltaTime units
   k ;counter in deltaTime units
   eps ;hysteresis
   sup ;upper actuation
   inf ;lower actuation
   #_RingBuffer r  ;reference
   #_RingBuffer y  ;output
   #_RingBuffer e  ;error
   #_RingBuffer u] ;input
  ICtrl
  (Diff [ctrl]
    (- (nth r -1) (nth y -1)))
  (Act [ctrl]
    (let [e-1 (nth e -1)]
      (cond
        (> e-1 eps) sup
        (< e-1 (- eps)) inf
        :else (nth u -1)))))

(defn new-relay
  ([sup inf eps n] (new-relay sup inf eps n map->RelayCtrl))
  ([sup inf eps n ctor]
   (let [ring (conj (ring-buffer n) 0)
         ryue (zipmap [:r :y :u :e] (repeat 4 ring))
         base {:n n :k 0 :eps eps :sup sup :inf inf}
         ctrl (ctor (merge base ryue))]
     (doto ctrl 
       (as/sets! u (conj (.u ctrl) sup))))))

;it captures the essence, since putting more arms
;can be seeing as a superposition
;the difficult bit is the interaction between arms
(defn roll-ctrl [^GameObject gob k]
  (as/let [(as/o :state [ref props]) gob
           y (.. gob transform rotation eulerAngles z)
           r (.. ref transform localRotation eulerAngles z)
           u (step-next (state gob k) r (- r (arc 90 (- r y))))
           p0 (+ u)
           p1 (- u)]
      (update-state (nth props 0) :spin/set + p0)
      (update-state (nth props 1) :spin/set + p1)))

;;constraining all but :z rotation it behaves linearly
;;as the relation between torque and angular momentum
;;since the two offsets result in a constant net torque
;;constraining only the :x rotation it swings
;;it would be interesting to investigate the position it ends
;;if I don't contrain the :x rotation it tumbles down
(def gangorra ;problem, cause it is stateful
  (x/renders
    (-> #'quad
        ; ((physx+ [:unstable-rotation])) ;todo
        ((x/freeze [:x :y :z] [:x :y]))
        (x/doto quad-1
          #(if (isa? (x/tag %2) ::lever)
             (with-cmpt %2 [tr Transform]
               (set! (.localRotation tr)
                     (aa 0 0 0 1))))
          #(as/let [(as/with-cmpt tr Transform) %]
             (set! (.position tr) (v3 0 10 0))
             (set! (.rotation tr) (aa 0 0 1 0))
             (hook- % :fixed-update :ctrl/euler)
             (hook+ % :fixed-update :ctrl/roll #'roll-ctrl)
             (state+ % :ctrl/roll (new-relay 10 -10 3 1))
             (state+ % :offset [600 600]))))
    :sansbox
    [::lever
     #'arm
     #'arm]))

; DOSTUFF

(def wait 20) ;seconds
(def qeu (atom (partition 2 (x/pool [nil qboth :sansbox]))))
(def prev (atom -1))

(defn main [^GameObject gob _]
  (let [now (int (mod Time/time wait))
        edge (and (zero? now) (not (zero? @prev)))]
    (reset! prev now)
    (when edge
      (when-let [[q s] (first @qeu)]
        (set! (.. s transform position) (v3 0 3 0))
        (set! (.. s transform rotation) (aa 5 1 0 0))
        (hook+ q :on-destroy ;todo: change the path name
          (x/fx (spit-watch q {:info "qboth"})
                (swap! qeu next)))
        (destroy q (dec wait))
        (destroy s (dec wait))
        (full-watch q)))))