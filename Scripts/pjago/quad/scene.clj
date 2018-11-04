(do (in-ns 'pjago.quad)
    (use 'pjago.watch)
    (require '[common.processing :as x :refer [render]]))
    ;(use 'common.repl)) ;;@DEV (to export, remove this)

;;PROOF THAT UNITY DOES NOT SIMULATE CROSS BETWEEN W

(defn T-scene
  ([] [nil :T :T])
  ([gob child])
  ([gob]
   (let [[T1 T2] (children gob)]
     (hook+ T1 :fixed-update :unstable-rotation #'missing-physx-udemy)
     (with-cmpt T1 [tr Transform rb Rigidbody]
       (set! (.position tr) (v3 -1 0 0)) 
       (set! (.angularVelocity rb) (v3 1000 0 0))
       (.AddTorque rb (v3 0 -10 0)))
     (with-cmpt T2 [tr Transform rb Rigidbody]
       (set! (.position tr) (v3 1 0 0))
       (set! (.angularVelocity rb) (v3 1000 0 0))
       (.AddTorque rb (v3 0 -10 0))))))

;;EDITOR
(do (in-ns 'pjago.quad) ;wut
    (import '[UnityEngine GameObject Screen GUI Color Rect]))

(def new-rect (memoize #(Rect. %1 %2 %3 %4)))

(defn label [[x y w h] txt]
  (GUI/Label (new-rect x y w h) txt))

(defn aero-info [^GameObject gob k]
  (when (= (count (objects-tagged (str ::quad))) 1)
    (let [{:keys [thrust drag spin speed]} (state gob)
          H (/ Screen/height 4)
          W (/ Screen/width 3)]
      ; (if-cmpt (object-tagged ":camera-main") [cam Camera]
      ;   (set! GUI/color Color/white)
      ;   (set! GUI/color Color/black))
      (label [0 0 W H] (format "T: %.2f" (float (* thrust gf:N))))
      (label [0 15 W H] (format "P: %.2f" (float (* drag gf:N))))
      (label [0 30 W H] (format "w: %.2f" (float spin)))
      (label [0 45 W H] (format "v: %.2f" (float speed))))))

(defn spin-info [^GameObject gob k]
  (when (= (count (objects-tagged (str ::quad))) 1)
    (let [props (state gob :props)
          n (count props)
          H (/ Screen/height 4)
          W (/ Screen/width 3)]
      ; (if-cmpt (object-tagged ":camera-main") [cam Camera]
      ;   (set! GUI/color Color/white)
      ;   (set! GUI/color Color/black))
      (set! GUI/color Color/black)
      (dotimes [i n]
        (let [u (state (nth props i) :spin/set)]
          (label [0 (* i 15) W H]
            (format "%d: %.2f" i (* 100.0 u))))))))

(defn ctrl-info [^GameObject gob k]
  (when (= (count (objects-tagged (str ::quad))) 1)
    (let [{:keys [props]} (state gob)
          H (/ Screen/height 4)
          W (/ Screen/width 1.17)]
      (set! GUI/color Color/black)
      (dotimes [i (count props)]
        (as/let [p (nth props i) {:keys [thrust drag]} (state p)]
          (label [W (* i 15) W H]
            (format "T(%d): %+2.2f" i (float (* thrust gf:N))))
          (label
            (if-cmpt (object-tagged ":camera-main") [cam Camera]
              [W (+ (- (* 4 H) 75) (* i 15)) W H]
              [(- W 100) (* i 15) W H])
            (format "P(%d): %+2.2f" i (float (* drag gf:N)))))))))

(defn state-info [^GameObject gob k]
  (when (= (count (objects-tagged (str ::quad))) 1)
    (as/let [H (/ Screen/height 4)
             W (/ Screen/width 3)]
      (as/let [e (.. gob transform rotation eulerAngles)
               h (.. gob transform position y)
               [x y] (if-cmpt (object-tagged ":camera-main") [cam Camera]
                       [0 (- (* 4 H) 75)]
                       [100 0])]
        (label [x y W H] (format "pitch: %.2f" (.-x e)))
        (label [x (+ y 15) W H] (format "yaw: %.2f" (.-y e)))
        (label [x (+ y 30) W H] (format "roll: %.2f" (.-z e)))
        (label [x (+ y 45) W H] (format "height: %.2f" h))))))

;; TCC

; deslizamento zero na interface com sólido (poeira no fan)
; teorema (?) do eixo intermediário (rotações instáveis)
; relação entre thrust, drag e a velocidade angular.
; como corrigir rotação e posição simultaneamente? (goto)
; aumento na estabilidade abaixando-se o centro de gravidade (?)

;; WATCH

(extend-protocol IWatch
  pjago.quad.IAero ; pjago.quad.IMutable would overwrite ICtrl
  (check [obj] (->> obj snapshot :arcadia.core/dictionary))
  (watch [obj] nil)
  pjago.quad.ICtrl
  (check [obj] (->> obj snapshot :arcadia.core/dictionary))
  (watch [obj]
    {:r (nth (.r obj) -1)
     :y (nth (.y obj) -1)
     :e (nth (.e obj) -1)
     :u (nth (.u obj) -1)
     :d (.d obj)})
  Double
  (check [obj] {:y obj})
  (watch [obj] {:y obj}) 
  Rigidbody
  (check [obj]
    (select-props obj
      [:drag :angular-drag :constraints
       :mass :center-of-mass :inertia-tensor]))
  (watch [obj]
    (select-props obj
      [:position :rotation :velocity :angular-velocity])))

(defn full-watch [gob & _]
  (.SetActive gob true)
  (state+ gob :rigidbody (cmpt gob Rigidbody))
  (state+ gob :aero (mapv #(state % :aero) (state gob :props))) ;specifics
  (reduce-kv watch+ gob
    #:ctrl
    {:_/aero 0 ; 0 only for it to check the props orientation
     :_/rigidbody 5001
     :euler 5001
     :yaw 5001
     :pitch 5001
     :roll 5001
     :height 5001}))
     ; :goto 1001
     ; :omni 1001
     ; :look 1001
     ; :plane 1001

;; ROLES

;todo: function to add hook, but in a specific order.
(def quad-ex
  {:fixed-update
   (merge #:ctrl {:offset 10 :input 15 :height 20 :euler 30 
                  :pitch 27 :yaw 28 :roll 29
                  :look 40 :goto 40 :omni 40
                  :break 50}
          #:ref {:height 1 :yaw 1 :pitch 1 :roll 1 :input 5}
          {:saturate 101 :balance 102
           :kinematic 200 :dynamic 300
           :physx 301 ::carry 302
           :disturbance 400
           :pjago.watch/watch 1000})})

(def quad-r
  {:disturbance {:fixed-update #'add-input :state [0.0 0.0 0.0 0.0]}
   :ctrl/offset {:fixed-update #'set-ctrl :state [0.0 0.0 0.0 0.0]}
   :ctrl/input {:fixed-update #'add-ctrl :state [0.0 0.0 0.0 0.0]}
   :ref/input {:fixed-update #'add-ref :state [0.0 0.0 0.0 0.0]}
   :physx {:fixed-update #'quad-physx}})

(def propeller-r
  {;:snap/connect {:on-transform-parent-changed #'snap-joint}
   ;:snap/break {:on-joint-break #'snap-joint}
   :spin/set {:state 0.0}
   :spin {:state 0.0}; :fixed-update #'kinematic}
   :speed {:state 0.0}; :fixed-update #'kinematic}
   :drag {:state 0.0}; :fixed-update #'dynamic}
   :thrust {:state 0.0}}); :fixed-update #'dynamic}})

(def prop4 ;todo: made this a parameter to a transducer
  ;@tcc: mostrar gangorra com esses aqui, comparar com a real
  ;justificar isso para indicar que a formula 0 está errada.
  ;talvez tenha sido um erro humano ou de medição
  ;contudo, a versão final utilizada será Martins3, para todos
  ;e o resultado também é próximo em termos de frequência
  ; [map->UIUCstatic map->UIUC map->UIUC map->UIUC map->UIUC])
  ; [map->Martins0 map->Martins3 map->Martins3 map->Martins3])
  (vec (repeat 4 map->Martins3)))
  ; (vec (repeat 4 map->Martins0)))
  ; (vec (repeat 4 map->Stables)))
  ; (vec (repeat 4 map->UIUCstatic)))
  ; (vec (repeat 4 map->UIUC)))

;tries to solve a proportional eq with precision eps, 
;through a binary search form inf to sup
(defn bin-search [eq eps inf sup]
  (loop [i 100 
         x (/ (+ inf sup) 2)
         inf inf
         sup sup]
    (if (pos? i)
      (let [error (eq x)]
        (cond
          (> error eps)
          (recur (dec i) (/ (+ inf x) 2) inf x)
          (< error (- eps))
          (recur (dec i) (/ (+ sup x) 2) x sup)
          :else
          x))
      (do (log "bin-search failed at" x) 
          x))))

(declare mass)

;solves for equal actuation on all propellers, that makes it hover
(defn off-zero [^GameObject q k]
  (let [m (.mass (ensure-cmpt q Rigidbody))
        weight (* (.magnitude Physics/gravity) m)
        props (state q :props)
        each (apply juxt
               (map (fn [p] (let [a (state p :aero)] #(.Thrust a % 0.0)))
                    props))
        equation #(- (reduce + (each (u->w %))) 
                     weight)
        solution (bin-search equation 0.0001 0.0 1.0)]
    (state+ q k [])
    (doseq [p props :let [a (state p :aero)]]
      (update-state q k conj solution))))

;; PREFABS

(defn propeller
  ([] [::propeller ::trail]) ;::trail
  ([gob child])
  ([gob]
   ;(snap-joint gob :snap/connect)
   (roles+ gob propeller-r)
   (state+ gob :aero (->UIUC 10.0 4.5 true))))

(defn arm
  ([gob child])
  ([] [::arm #'propeller ::motor ::esc ::forearm ::elbow ::hand])
  ([gob])) ;(hook+ gob :on-transform-children-changed :loose #'loose-props)))

(derive ::motor ::carry)
(derive ::batt ::carry)
(derive ::esc ::carry)

;;PJSX
(defprotocol Geometry
  (vol [this])
  (tensor [this])) ;analogous to com

(defn root-seq [^GameObject gob]
  (tree-seq parent #(cons (parent %) nil) gob))

(defn colliders 
  ([^GameObject gob] (colliders gob Collider))
  ([^GameObject gob of-type]
   (sequence (comp (keep #(seq (cmpts % of-type))) cat
                   (filter #(.enabled %)))
             (tree-seq any? #(remove (fn [x] (cmpt x Rigidbody)) (children %)) 
                       gob))))

(defn scale [^GameObject gobj] ;todo: recursive, and memoize? lame
  (if (= gobj (.. gobj transform root gameObject))
    (.. gobj transform localScale)
    (transduce (map #(.. % transform localScale)) 
               (completing #(v3map * %1 %2))
               (v3 1)
               (root-seq gobj))))

(defn com
  ([gc]
   (if-not (instance? GameObject gc)
     (let [gob (gobj gc)]
       (v3+ (.. gob transform position)
         (qq* (.. gob transform rotation)
              (v3map * (scale gob) (.center gc)))))
     (com gc Collider)))
  ([^GameObject gob of-type]
   (if-let [cs (seq (colliders gob of-type))]
     (let [w (map #(v3* (com %) (vol %)) cs)]
       (v3* (reduce v3+ w) (/ (vol gob))))
     (.. gob transform position))))

(defn pax ;parallel axis tensor (without the mass)
  ([gc] (pax gc (.. (gobj gc) transform position)))
  ([gc ^Vector3 here]
   (let [o (v3- (com gc) here)
         dot (Vector3/Dot o o)
         out (M/Outer o o)]
     (M/Add (trs (v3 0) (qt) (v3 dot)) (M/Mul out (float -1.0))))))

;todo: controller unstable
;constrain things
;fix this
(extend-protocol Geometry ;cannot devie by zero at completing!!
  GameObject ;todo? calculate scale at gobj level, memoizing
  (vol [this]
    (transduce (map vol) + (colliders this)))
  (tensor [this]
    (let [undo (Quaternion/Inverse (.. this transform rotation))]
      (transduce (map #(M/Mul (M/Add (M/Rot (.. (gobj %) transform rotation)
                                            (tensor %))
                                     (pax % (com this)))
                              (vol %)))
                 (completing #(M/Add %1 %2)
                             #(M/Mul (M/Rot undo %1) (/ (vol this))))
                 (trs (v3 0) (qt) (v3 0))
                 (colliders this))))
  BoxCollider
  (vol [this]
    (.magnitude (v3map * (scale (gobj this)) (.size this))))
  (tensor [this]
    (let [w (v3map * (scale (gobj this)) (.size this))
          v (v3- (v3 (Vector3/Dot w w)) (v3map (fn [x] (Math/Pow x 2)) w))]
      (trs (v3 0) (qt) (v3* v 1/12))))
  CapsuleCollider ;aka Cylinder
  (vol [this]
    (as/let [d (.direction this)
             (as/o :props [x y z]) (scale (gobj this))
             [x y z] (case d 0 [z x y] 1 [x y z] 2 [y z x])
             h (* (.height this) y)
             r (.radius this)
             a (* r x)
             b (* r z)]
      (* h m/pi a b)))
  (tensor [this] ;todo: x, and z direction.
    (as/let [(as/o :props [x y z]) (scale (gobj this))
             h (* (.height this) y)
             r (.radius this)
             a (* r x)
             b (* r z)
             Ix (/ (+ (* 3 a b) (* h h)) 12)
             Iy (* 0.5 a b)]
      (trs (v3 0) (qt) (v3 Ix Iy Ix)))))

;does not attempts to diagonalize, by finding an appropiate rotation
;it just asserts that the off diagonal elements are already close to 0
;in respect to the diagonal elements.
;todo: http://melax.github.io/diag.html
(defn inertia-tensor [^GameObject gob]
  (let [p (.. gob transform position)
        r (.. gob transform rotation)
        rbx (keep #(cmpt % Rigidbody))
        cmx (map #(let [g (gobj %)]
                    (M/Mul (M/Add (M/Rot r (tensor g)) (pax g p)) (.mass %))))
        mat (transduce (comp rbx cmx) 
                       (completing #(M/Add %1 %2) #(M/Rot (Quaternion/Inverse r) %1)) 
                       (trs (v3 0) (qt) (v3 0))
                       (gobj-seq gob))]
    (assert (m/eps= (m/pow (determinant mat) 2)
                    (determinant (m* mat (.transpose mat)))
                    1e-8)
            (str "Something went wrong with the tensor\n"
                 [(.get_Item mat 0 0) (.get_Item mat 1 1) (.get_Item mat 2 2)]
                 "\nCheck yo symetry!"))
    (v3map #(Math/Round % 5)
      (v3 (.get_Item mat 0 0) (.get_Item mat 1 1) (.get_Item mat 2 2)))))

(defn center-of-mass [^GameObject gob] ;@tcc
  (let [tr (.transform gob)
        rbx (keep #(cmpt % Rigidbody))
        cmx (map #(as-> (.mass %) m [m (v3* (com (gobj %)) m)]))
        step (fn ;todo: faster
               ([[acc v3acc] [om oc]]
                [(+ acc om) (v3+ oc v3acc)])
               ([[acc v3acc]]
                (v3* v3acc (/ acc))))]
    ;(v3map / ;because the center-of-mass is on local units, and com is on world units
      (v3map #(Math/Round % 4)
        (qq* (Quaternion/Inverse (.rotation tr))
             (v3- (transduce (comp rbx cmx) step [0 (v3 0)] (gobj-seq gob))
                  (.position tr))))))
    ;  (v3 1.0 -1.0 1.0) ;@weird but it seems to work
    ;  (.lossyScale tr)))

(defn mass [^GameObject gob] ;@tcc
  (let [rbx (keep #(cmpt % Rigidbody))]
    (transduce (comp rbx (map #(.mass %))) + (gobj-seq gob))))

;todo? maybe cache the pieces like I do with weight-carry
(defn compute-tensor [^GameObject gob & _] ;@tcc
  (let [rb (ensure-cmpt gob Rigidbody)]
    (if-let [m (state gob :self-mass)]
       (set! (.mass rb) m))
    (set! (.centerOfMass rb) (center-of-mass gob))
    (set! (.inertiaTensorRotation rb) (qt)) ;todo: diagonalize
    (set! (.inertiaTensor rb) (inertia-tensor gob))
    (set! (.mass rb) (float (mass rb)))))

(defn weight-carry [^GameObject gob k]
  (doseq [rb (state gob k)]
    (if (.activeInHierarchy (gobj rb))
      (.AddForceAtPosition (cmpt gob Rigidbody)
        (v3* Physics/gravity (.mass rb))
        (com (gobj rb))))))

(defn quad
  ([] [::quad (repeat 4 #'arm) ::base ::lshell ::ushell])
  ([gob child])
  ([gob]
   (let [child-seq (rest (gobj-seq gob))
         {:keys [::arm ::propeller]}
         (group-by x/tag child-seq)]
     (if-not (state gob :ref) (state+ gob :ref gob))
     (if (second arm) (mirror! arm (v3 0) up))
     (dotimes [i (count propeller)]
       (let [p (nth propeller i)
             a (state p :aero)
             a ((nth prop4 i) 
                (assoc (:arcadia.core/dictionary (snapshot a)) 
                  :clockwise (zero? (mod i 2))))]
         (state+ p :aero a)))
     (roles+ gob quad-r)
     (state+ gob :props propeller))))

(defn physx+ [ks]
  #(x/doto %
     (fn [gob]
       (if-cmpt gob [rb Rigidbody]
         (when-not (.isKinematic rb)
           (doseq [k ks]
             (hook+ gob :fixed-update k #'missing-physx))))
       gob)))

(defn focus-quad [^GameObject gob k]
  (if-let [follow (state gob :follow)]
    (if-not (obj-nil (.lock follow))
      (if-let [q (object-tagged (str ::quad))]
        (with-cmpt gob [cam Camera]
          (mario-cam! gob q))))))

(defn focus-steal [cam-tags]
  (partial x/complete
    #(doseq [cam-tag cam-tags]
       (if-let [cam (object-tagged (str cam-tag))]
         (mario-cam! cam %)))))

(derive ::elbow ::axis)
(derive ::hand ::axis)
(derive ::forearm ::axis)
(derive ::rotor ::bldc)
(derive ::stator ::bldc)

(defn paint ;todo: xf with api similar to tree-seq, name it tricycle
  ([materials n hkey] (paint materials n hkey false))
  ([materials n hkey gob-seq?]
   (let [m (cycle (map #(x/resource % Material) materials))
         i (volatile! 0)]
     (x/postwalk
       #(cond
          (isa? % ::arm)
          (do (vswap! i inc) %)
          (isa? % hkey)
          ((x/paint (nth m (quot @i n)) gob-seq?) (x/wrap %))
          :else %)))))

(def camera-main
  (let [default (qv* (angle-axis 45 up) (v3 0 -0.05 15))]
   ^{::x/init :camera-main}
    {:focus {:state default}
     :follow {:late-update #'follow-pos
              :state (->LockPos nil default)}}))

;funny that this works similar to a prefab.
;if I set! the default follow of camera-main, it will set! on camera-side too
;that's why in common.processing, we walk snapshot and mutable on state
(def camera-side
  (with-meta camera-main {::x/init :camera-side}))

;very similar but with another default...
(def last-camera
  (let [default (v3 0 0.85 -1.5)]
   ^{::x/init :last-camera}
    {:focus {:state default}
     :follow {:late-update #'follow-pos
              :state (->LockPos nil default)}}))

;and yet again... so this should be a transducer that inits the default state
;from the prefab initial position! @TODO
(def other-camera
 ^{::x/init :other-camera}
  {:focus {:state (v3 0 0 -10)}
   :follow {:late-update #'follow-pos
            :state (->LockPos nil (v3 0 0 -10))}})

(defn preps
  ([] [nil :other-camera :another-camera])
  ([graph child]
   (if-cmpt child [cam Camera]
     (set! (.backgroundColor cam) Color/white)))
  ([graph]
   (run! #(.SetActive % false) (rest (gobj-seq graph)))))

;TODO: REPLICATE CAMERA-MAIN AS CAMERA HELP, rotate 45, put on the right,
;then move the values down and to the center, so the quad doesn't block them
;finally, it should have two 45° views and two 0° views
(defn background
  ([]
   [nil (repeat 4 ::direct-light) :other-light :pinpoint])
  ([_ child]
   (if-cmpt child [cam Camera]
     (set! (.backgroundColor cam) Color/white)))
  ([scene]
   (mirror! (::direct-light (group-by x/tag (gobj-seq scene))) 
            (v3 0) up)))

;; todo: find out if this will compile
(def it 
  #(clojure.pprint/pprint
     (mapv (fn [rb] (conj (to-angle-axis (.inertiaTensorRotation rb)) (.inertiaTensor rb)))
           (mapv (fn [x] (cmpt x Rigidbody)) (objects-tagged (str ::quad))))))

(def cm 
  #(clojure.pprint/pprint
     (mapv (fn [rb] [(.mass rb) (.centerOfMass rb)])
           (mapv (fn [x] (cmpt x Rigidbody)) (objects-tagged (str ::quad))))))

(when-let [clear (resolve 'common.repl/clear-all!)]
  (def clean-background
    (alter-var-root #'background
      #(x/doto % (fn [] (clear))))))

;; VERSIONING

(defn quad-0 "aero & ctrl gui"
  ([] [#'quad ::lever])
  ([gob child]
   (if (isa? (x/tag child) ::lever)
     (state+ gob :ref (first (children child)))) ;avoid skewing
   (if (and Application/isPlaying (isa? (x/tag child) ::base))
     (destroy child 3)))
  ([gob]
   (doseq [p (state gob :props)]
     (hook+ p :on-gui :aero #'aero-info)
     (set! (.enabled (cmpt p (hook-types :on-gui))) false))
   (off-zero gob :ctrl/offset)
   (hook+ gob :on-gui :spin #'spin-info)
   (hook+ gob :on-gui :ctrl #'ctrl-info)
   (hook+ gob :on-gui :state #'state-info)
   (set! (.enabled (cmpt gob (hook-types :on-gui))) false)))
   
(defn quad-1 "centralize kinematics and dynamics"
  ([] [#'quad-0 ::batt])
  ([gob child])
  ([gob]
   (hook+ gob :fixed-update :kinematic #'kinematico)
   (hook+ gob :fixed-update :dynamic #'dynamico)
   (doseq [p (state gob :props)]
     (destroy-immediate (cmpt p Joint))
     (destroy-immediate (cmpt p Rigidbody))
     (clear-hook p :fixed-update))))

(derive ::forearm-top ::forearm)
(derive ::hand-top ::hand)

(def ^{:doc "improve visuals, refine arm collider"}
  quad-2
  (->> (x/wrap [#'quad-1]) ;this is funky
       ((x/postwalk #(if (and (sequential? %)
                              (isa? (first %) ::arm))
                       (concat % [::forearm-top ::hand-top])
                       %)))
       (x/step
         #(when (isa? (x/tag %2) ::arm) 
            (cmpt- %2 BoxCollider)
            (.SetActive (.. %2 transform (Find "hand") gameObject) 
                        false)
            (.SetActive (.. %2 transform (Find "forearm") gameObject) 
                        false)))
       ((paint [:plastic-white :plastic-red] 2 ::axis))
       ((paint [::M3 ::M0 ::M1 ::M2] 1 ::motor true))))
       ;((focus-steal [:other-camera :last-camera])))) ;nobody expects

(def actuator-delay 1) ;in sampling units (doesn't count ioDelay)

(defn quad-3 "transforms to align x and y, adds delay"
  ([] [((x/transform-to (v3 0) (angle-axis 45 up)) #'quad-2)])
  ([gob child]
   (if (isa? (x/tag child) ::lever)
     (set! (.. child transform localRotation) (qt))))
  ([gob]
   (doseq [p (::propeller (group-by x/tag (gobj-seq gob)))]
     (if-not (zero? actuator-delay)
       (state+ p :spin/fifo
         (into (ring-buffer actuator-delay)
               (repeat actuator-delay 0.0)))))))

(defn quad-4 "own center of mass, own inertia tensor"
  ([] [#'quad-3])
  ([gob child])
  ([gob]
   (with-cmpt gob [rb Rigidbody]
     ; (state+ gob ::carry #{})
     ; (hook+ gob :on-transform-children-changed ::carry #'compute-tensor)
     ; (hook+ gob :fixed-update ::carry #'weight-carry)
     ; (doseq [child (rest (gobj-seq gob))]
     ;   (if (isa? (x/tag child) ::carry)
     ;     (update-state gob ::carry conj
     ;       (ensure-cmpt child Rigidbody))))
     (if-let [m (state gob :self-mass)] ;had to do this for now
       (set! (.mass rb) m)
       (state+ gob :self-mass (.mass rb)))
     (set! (.centerOfMass rb) (center-of-mass gob))
     (set! (.inertiaTensorRotation rb) (qt 1 0 1 0))
     (set! (.inertiaTensor rb) (inertia-tensor gob))
     (set! (.mass rb) (mass gob)) ;loosing idempotency @bug
     (off-zero gob :ctrl/offset))))
     ; (doseq [child (rest (gobj-seq gob))]
     ;   (if-cmpt child [_ Rigidbody]
     ;     (cmpt- child Rigidbody))))))

(defn balance-props [^GameObject gob k] ;@bug?
  (let [props (state gob :props)]
    (doseq [[k v] (state gob :balance)]
      (update-state (nth props k) :spin/set
        #(+ (* % (v 0)) (v 1))))))

(def balance4
  {Martins0 [0.8162 28.5875]})

;@WIP
(defn quad-5 "accounts for asymetry on the propellers/motors"
  ([] [#'quad-4])
  ([gob child])
  ([gob] ;todo: make (tree-map gob) that graphs by parents
   (let [balance #(balance4 (type (state % :aero)))
         idx+ab #(if (some? %2) [%1 %2])]
     (hook+ gob :fixed-update :balance #'balance-props)
     (->> (gobj-seq gob) (group-by x/tag) ::propeller
          (into {} (comp (map balance) (keep-indexed idx+ab)))
          (state+ gob :balance)))))

(defn qgen []
  (comp #(x/doto % (fn [gob] (full-watch gob))) ; x/doto is very specific about arity
        (x/execution-order quad-ex)))

(def qoff
 ^{::x/init #'quad-4 ::x/renderer #'qgen}
  {:ctrl/offset {:state [0.0 0.0 0.0 0.0]}})

(def qon
 ^{::x/init #'quad-4 ::x/renderer #'qgen} 
 {})

(def h-ziegler (partial new-pid 0.061931768260432 5.629147320464203 1.407286830116051 1)) ;nichols(Gp(end), w(end), 'PID');
(def h-astron (partial new-pid 0.109480933269563 4.911199987099938 1.227799996774984 1)) ;astron(Gp(end), 1.0*exp(-0.75j*pi), wp(end), 0.25);
(def pidw (partial new-pid (* 4 0.109480933269563) (* 4 4.911199987099938) 1.227799996774984 1 map->PIDCtrl)) ;map->PI_DWCtrl :umax 1.0 :umin -1.0))


;;;;;;;;;;;;;;;;;;; @tcc
;these controllers should be tuned to have similar settling time, and do their best to keep a low overshoot @todo

;; roll & pitch

; OBS: Gs has to be multiplied by 180/pi to convert from rad to degrees

;[ku, ~, wu, ~] = margin(Gs)
;[kp, ti, td] = ziegler_nichols(-1/ku, wu, 'PD')
;C = pid(kp, kp/ti, kp*td)
(def zn (partial new-pid 0.001842035780390 m/inf 0.431048434398831 1 map->PIDCtrl))

;pa = harmmean(Gs.p{1}(2:3))/2;
;rp = -0.8*pa; % time constant 25% bigger than approximation
;[kp, ti, td] = pplace(Ga, conv([1 10*rp], [1 2*rp (rp/0.4)^2]), 'I+PD')
;C = pid2(kp, kp/ti, kp*td, 0.0, 0.0, 0.0)
;[C1, C2] = getComponents(C, 'feedback')
(def i-pd (partial new-pid 0.009396065827870 0.584337785020016 0.580106195293490 1 map->I_PDCtrl))

;[C1, C2] = ctrl(Gs, 'CMP+D', [1 2.1 3.4 2.7 1].*[1 3.8 3.8^2 3.8^3 3.8^4])
(def lead-d (partial new-comp-d 0.001444015892615 0.480775289467486 0.917906266301724 0.005614255408370 1))

;[C1, C2] = ctrl(Gs, 'CMP+D', [1 2.1 3.4 2.7 1].*[1 5.8 5.8^2 5.8^3 5.8^4])
(def lead-d1 (partial new-comp-d -0.028700360631945 1.136070872355795 0.843951711403380 0.014943137876787 1)) ;savage
;[C1, C2] = ctrl(Gs, 'CMP+D', [1 3 4 3 1].*[1 2.5 2.5^2 2.5^3 2.5^4])
(def lead-d2 (partial new-comp-d 0.001024111720224 0.862195893123687 0.843951711403380 0.001428864287386 1)) ;soft
;[C1, C2] = ctrl(Gs, 'CMP+D', [1 2.1 3.4 2.7 1].*[1 3.8 3.8^2 3.8^3 3.8^4])
(def lead-d3 lead-d) ; ~ 1s
;[C1, C2] = ctrl(Gs, 'CMP+D', conv([1 2*5], [1 2*0.7*1.5 1.5^2]))
(def lead-d4 (partial new-comp-d 0.003886770646291 0.974313512649167 0.823590247205729 -3.798982600623447e-05 1)) ;push

;; yaw

;[ku, ~, wu, ~] = margin(Gs)
;[kp, ti, td] = ziegler_nichols(-1/ku, wu, 'PD')
(def zny (partial new-pid 0.003322763393886 m/inf 0.433623004645392 1 map->PIDCtrl))

;pa = harmmean(Gs.p{1}(2:3))/2;
;rp = -0.8*pa; % time constant 25% bigger than approximation
;[kp, ti, td] = pplace(Ga, conv([1 5*rp], [1 2*rp (rp/0.8)^2]), 'I+PD')
;C = pid2(kp, kp/ti, kp*td, 0.0, 0.0, 0.0)
;[C1, C2] = getComponents(C, 'feedback')
(def i-pdy (partial new-pid 0.006330202825293 2.24947492764429 0.766272596272631 1 map->I_PDCtrl))

;[C1, C2] = ctrl(Gs, 'CMP+D', [1 3 4 3 1].*[1 2.5 2.5^2 2.5^3 2.5^4])
(def lead-dy (partial new-comp-d 0.002661676920918 0.910079857165203 0.932226004122239 0.002446185998505 1))

;[C1, C2] = ctrl(Gs, 'CMP+D', [1 2.1 3.4 2.7 1].*[1 3.8 3.8^2 3.8^3 3.8^4])
(def lead-dy1 (partial new-comp-d 0.005485960978318 0.768220298412103 0.923319454323895 0.009478904119835 1))
;[C1, C2] = ctrl(Gs, 'CMP+D', [1 3 4 3 1].*[1 2.5 2.5^2 2.5^3 2.5^4])
(def lead-dy2 lead-dy)
  
;; height

;[ku, ~, wu, ~] = margin(Gs)
;[kp, ti, td] = ziegler_nichols(-1/ku, wu, 'PD')
(def znh (partial new-pid 0.122660421839441 m/inf 1.052617845504647 1 map->PIDCtrl))

;pa = harmmean(Gs.p{1}(2:3))/2;
;rp = -0.8*pa; % time constant 25% bigger than approximation
;[kp, ti, td] = pplace(Ga, conv([1 5*rp], [1 2*rp (rp/0.8)^2]), 'I+PD')
;C = pid2(kp, kp/ti, kp*td, 0.0, 0.0, 0.0)
;[C1, C2] = getComponents(C, 'feedback')
(def i-pdh (partial new-pid 0.166070006068346 6.477652977493942 2.187678513670067 1 map->I_PDCtrl))

;[C1, C2] = ctrl(Gs, 'CMP+D', [1 2.1 3.4 2.7 1].*[1 2 2^2 2^3 2^4])
(def lead-dh (partial new-comp-d 0.144238119371038 0.693654872031150 0.956003217835199 1.22168 1))

;[C1, C2] = ctrl(Gs, 'CMP+D', [1 2.1 3.4 2.7 1].*[1 3.8 3.8^2 3.8^3 3.8^4])
;minimum phase boiiiii @not at tcc
(def lead-dh1 (partial new-comp-d -9.964760595714885 1.055673526307153 0.923319454323895 0.843951711403380 1))
;[C1, C2] = ctrl(Gs, 'CMP+D', [1 2.1 3.4 2.7 1].*[1 2 2^2 2^3 2^4])
(def lead-dh2 lead-dh)

;;;;;;;;;;;;;;;;;;;

(def qheight
  (merge qon
    {:ctrl/height {:fixed-update #'height-ctrl
                   :state (lead-dh)}}))

(def qheight-relay
  (update qheight :ctrl/height assoc :state (new-relay 0.4346 0.4146 0 1)))

;not @tcc

;think if the partial think can be avoided @todo
(def pid1 (partial new-pid 5.6041604e-04 0.081425178 0.58343057 1))
(def pid2 (partial new-pid 7.25010e-05 0.01163899 3.0591298 1))
(def pida (partial new-pid 1.0e-04 0.5 0.5 1))
(def pd1 (partial new-pid 0.001203534898 m/inf 0.663698895 1))

(def pid (partial new-pid 5.60503848e-04 0.08143446 0.5833592 1)) ;nope
(def lcp (partial new-pid 5.60503848e-04 0.08143446 0.5833592 1 map->LCpCtrl :kr 2.8675 :ku (m/exp (* -2.5 0.02)) :kf (m/exp (* -5.0 0.02))))
(def pd (partial new-pid 0.0012010108 m/inf 0.545360738 1 map->PI_DCtrl))
;(def pidw (partial new-pid (* 4 0.109480933269563) (* 4 4.911199987099938) 1.227799996774984 1 map->PI_DWCtrl :umax 1.0 :umin -1.0))

;astron, maybe put a little more though on b and c , or maybe just show the oscilations
(def euler-pid
  {:x [map->PIDCtrl]
   :y [map->PIDCtrl]
   :z [map->PIDCtrl]
   :e [map->EulerCtrl]})

(def euler-pi_d
  {:x [map->PI_DCtrl]
   :y [map->PI_DCtrl]
   :z [map->PI_DCtrl]
   :e [map->Euler2Ctrl]})

(def euler-lcp
  {:x [map->LCpCtrl :kr 2.8675 :ku 0.951229424 :kf 0.904837418]
   :y [map->PI_DCtrl]
   :z [map->LCpCtrl :kr 2.8675 :ku 0.951229424 :kf 0.904837418]
   :e [map->Euler2Ctrl]})

(def qeuler ;@tcc
  (let [x (lead-d) y (lead-dy) z (lead-d)]
    (merge qon
      #:ctrl
      {:pitch {:state x}
       :yaw {:state y}
       :roll {:state z}
       :euler {:state (apply new-euler x y z (euler-pi_d :e))
               :fixed-update #'euler-ctrl}})))

(def qyaw-relay ;@strange
  (let [x (new-relay 0 0 0 1)
        y (new-relay 5 -5 0 1)
        z (new-relay 0 0 0 1)]
    (merge qon
      #:ctrl
      {:pitch {:state x}
       :yaw {:state y}
       :roll {:state z}
       :euler {:state (apply new-euler x y z (euler-lcp :e))
               :fixed-update #'euler-ctrl}})))

;sometimes even though the height controller is stable,
;and the euler controller is stable, when you add them the result
;is not stable anymore
;e.g. tilt   (new-pid 0.02937 1.9799 0.4950 1)
;     yaw    (new-pid 0.03525 0.8327 0.2082 1)
;     height (new-pid 50.0 1.8 0.6 1 map->PI_DWCtrl
;                     :umax 80
;                     :umin 20}})))

(def qboth ;@tcc
  (merge qeuler qheight))
    ;{:ctrl/offset {:state [0.0 0.0 0.0 0.0]}}))

;;TODO: BOT, MID, TOP MOUNT BATTERY

(def qlook ;doesn't uses a pid (tá estranho também, melhor deixar de fora)
  (merge qboth
    {:ctrl/look {:fixed-update #'plane-ctrl
                 :state (new-pid 3.0 5.0 1.0 1)}}))

;applying (new-relay 5.0 -5.0 1.0 1) as the omni controller
;and positioning it at (v3 1 0 0), it stays above the x axis for a while
;and then it deviates in the z direction, following a circular movement at the end

(def qomni
  (merge qboth
    #:ctrl
    {:omni {:fixed-update #'plane-ctrl
            :state (new-pid 0.3 5.0 1.0 1 map->PIDWCtrl
                            :umax max-tilt
                            :umin (- max-tilt))}}))

(def qomni-relay
  (update qomni :ctr/omni assoc :state 
          (new-relay 1.0 -1.0 1.0 1)))

(def qgoto ;@not a tcc
  (merge qboth
    #:ctrl
    {:goto {:state (new-pid 3.0 100 1.0 1)
            :fixed-update #'plane-ctrl}}))

(def qbreak ;@maybe NOT
  (merge qboth
    #:ctrl
    {:break {:state (new-pid 2.0 m/inf 5.0 1)
             :fixed-update #'break-ctrl}}))
  
;; RENDER

(def simple-prop
  (x/renders
    (x/doto #'quad #'quad-1
      #(state+ % :ctrl/offset [929])
      #(hook- % :fixed-update :ctrl/euler))
    :sansbox
    [(x/renders #'propeller :pjago.quad.prop/fixed nil)]))

(def armed-prop
  (x/renders
    (x/doto #'quad quad-1
      #(state+ % :ctrl/offset [929])
      #(hook- % :fixed-update :ctrl/euler))
    :sansbox
    [#'arm]))

;; TCC

(defn wrand ;@ants.clj
  "given a vector of slice sizes, returns the index of a slice given a
  random spin of a roulette wheel with compartments proportional to
  slices."
  [slices]
  (let [total (reduce + slices)
        r (rand total)]
    (loop [i 0 sum 0]
      (if (< r (+ (slices i) sum))
        i
        (recur (inc i) (+ (slices i) sum))))))

;it captures the essence, since putting more arms
;can be seeing as a superposition
;the difficult bit is the interaction between arms
(defn pitch-ctrl [^GameObject gobj k] ;@tcc
  (let [props (state gobj :props)
        r (.. (state gobj :ref) transform localRotation eulerAngles x)
        y (.. gobj transform rotation eulerAngles x)
        r (arc -180 r)
        u (step-next (state gobj k) (- r) (arc -180 y))
        p0 (+ (/ u 2))
        p2 (- (/ u 2))]
    (update-state (nth props 0) :spin/set + p0)
    (update-state (nth props 2) :spin/set + p2)))
 
(defn only-even [^GameObject gob k]
  (let [props (state gob :props)]
    (state+ (nth props 1) :spin/set 0.0)
    (state+ (nth props 3) :spin/set 0.0)))

(defn pitch-step [^GameObject gob k]
  (let [rt (.transform (state gob :ref)) 
        y (arc -180 (.. gob transform rotation eulerAngles x))
        a (.. rt localRotation eulerAngles)
        t (or (state gob :time/pitch) 0)]
    (if (> (- Time/time t) 5)
      (let [delta 1.0
            slices (state gob k)
            n (count slices)
            _ (assert (odd? n) "reference for step must be odd")
            h (quot n 2)
            i (+ h (max (- h) (min h (Math/Round (/ y delta) 0))))
            slices (assoc slices i 0)
            i (wrand slices)
            c (dec (slices i))
            r (* delta (- i h))]
        (state+ gob :time/pitch Time/time)
        (set! (.localRotation rt) (euler (v3 r (.y a) (.z a))))
        (update-state gob k assoc i (if (zero? c) 1 c))))))

;feeling lazy
(defn roll-step [^GameObject gob k]
  (let [rt (.transform (state gob :ref)) 
        y (arc -180 (.. gob transform rotation eulerAngles z))
        a (.. rt localRotation eulerAngles)
        t (or (state gob :time/roll) 0)]
    (if (> (- Time/time t) 5)
      (let [delta 1
            slices (state gob k)
            n (count slices)
            _ (assert (odd? n) "reference for step must be odd")
            h (quot n 2)
            i (+ h (max (- h) (min h (Math/Round (/ y delta) 0))))
            slices (assoc slices i 0)
            i (wrand slices)
            c (dec (slices i))
            r (* delta (- i h))]
        (state+ gob :time/roll Time/time)
        (set! (.localRotation rt) (euler (v3 (.x a) (.y a) r)))
        (update-state gob k assoc i (if (zero? c) 1 c))))))

;;constraining all but :z rotation it behaves linearly
;;as the relation between torque and angular momentum
;;since the two offsets result in a constant net torque
;;constraining only the :x rotation it swings
;;it would be interesting to investigate the position it ends
;;if I don't contrain the :x rotation it tumbles down
;problem! can't freeze x or y, because of the 45° rotation
;maybe try to import fbx again but with arms aligned
;keep the meta files, maybe the prefabs will stil work

;the mass was exchange from the quad distribution to the center as
;needed to reduce to the inertia and match the critical frequency
;this process inteds to account for the different densities
(defn gangorra [q]
  (x/renders ;@tcc
    (-> (x/wrap q)
        ; ((physx+ [:unstable-rotation])) ;todo
        ; (x/doto #(cmpt+ % HingeJoint))
        ((x/freeze [:y] [:y])) ;let's freeze, not hinge
        ;((focus-steal [:camera-main]))
        ((x/execution-order
           (update quad-ex :fixed-update assoc
                   :ctrl/pitch 31
                   :freeze 1000))))))
    ; :sansbox ;;todo: find out why it breaks
    ; [#'arm
    ;  #'arm]))
 
; DOSTUFF

(defn sansbox
  ([] [:sansbox])
  ([gob child])
  ([gob]
   (set! (.. gob transform position) (v3 0 3 0))
   (set! (.. gob transform rotation) (aa 5 0 0 0))))

; (def wait 20) ;seconds
; (def qeu (atom (x/pool [nil qboth sansbox])))
; (def prev (atom -1))

; (defn main [^GameObject gob _]
;   (let [now (int (mod Time/time wait))
;         edge (and (zero? now) (not (zero? @prev)))]
;     (reset! prev now)
;     (when edge
;       (when-let [[q s] (first @qeu)]
;         (hook+ q :on-destroy
;           (x/fx (spit-watch q) (swap! qeu next)))
;         (destroy q (dec wait))
;         (destroy s (dec wait))
;         (full-watch q)))))

;; WIND

(defn wind-force ;todo? integrate pressure over collider
  ([^GameObject gob ^Collider c k]
   (let [upfn (case k :enter conj :exit disj)
         pool (state gob :pool)]
     (if-cmpt c [rb Rigidbody]
       (update-state gob :pool upfn rb))))
  ([^GameObject gob _]
   (as/let [(as/with-cmpt wz WindZone) gob
            strength (/ (.windMain wz) 0.1 gf:N)
            direction (.. gob transform forward)] ;@todo
     (doseq [rb (objects-tagged (str ::quad))]; (state gob :pool)]
       (.AddForce (cmpt rb Rigidbody) 
                  (v3* direction strength)
                  Space/World)))))

(def wind-zone
 ^{::x/init ::wind-zone}
  {:forward {:state forward}
   :pool {:state #{}}
   :enter {:on-trigger-enter #'wind-force}
   :exit {:on-trigger-exit #'wind-force}
   :push {:fixed-update #'wind-force}})

(defn wind-scene
  ([] [nil wind-zone (repeat 4 ::tree)])
  ([gob]
   (let [{trees ::tree [wind] ::wind-zone} 
         (group-by x/tag (gobj-seq gob))]
     (run! #(child+ wind % true) trees)
     (if (second trees)
       (mirror! trees (v3 0) up))))
  ([gob child]
   (if (isa? (x/tag child) ::wind-zone)
     (set! (.. child transform forward) 
           (state child :forward)))
   (if (isa? (x/tag child) ::tree)
     (.. child transform (Translate (v3 3 0 0))))))

(defn xz ;@tcc
  ([q] (xz q (v3 1 0.5 0)))
  ([q ^Vector3 position]
   (x/renders #'wind-scene
     [(x/doto ((x/freeze [:y]) (x/wrap q))
        #(do (set! (.. % transform position) position)
             (hook- % :fixed-update :ctrl/height)
             (hook+ % :on-enable :ctrl/offset #'off-zero)
             (full-watch %)))])))

;; HEIGHT

;too bad the i/o is mixed with the reference generator!
;I should refactor this so that it saves the index as well @todo
(defn height-step [^GameObject gob k]
  (let [rt (.transform (state gob :ref)) 
        y (.. gob transform position y)
        r (.. rt localPosition y)
        t (or (state gob :time/height) 0)
        o 0.2]
    (if (> (- Time/time t) 10)
      (let [delta 0.1
            slices (state gob k)
            n (dec (count slices))
            i (max 0 (min n (Math/Round (/ (- y o) delta) 0)))
            slices (assoc slices i 0)
            i (wrand slices)
            c (dec (slices i))
            r (- (* -1 delta (dec i)) o)]
        (state+ gob :time/height Time/time)
        (set! (.localPosition rt) (v3 0 r 0))
        (update-state gob k assoc i (if (zero? c) 1 c))))))

(defn height-scene ;find out why I did this @todo
  ([] [nil wind-zone ::tree])
  ([gob] ;repeat
   (let [{trees ::tree [wind] ::wind-zone} 
         (group-by x/tag (gobj-seq gob))]
     (run! #(child+ wind % true) trees)))
  ([gob child]
   (if (isa? (x/tag child) ::wind-zone)
     (set! (.. child transform forward) up))
   (if (isa? (x/tag child) ::tree)
     (set! (.. child transform position)
           (v3 0.25 -0.5 -0.25)))))
           ;(v3 0.25 -0.5 -0.25)))))

(defn y [q] ;@tcc
  (x/renders #'height-scene
    [(x/doto ((x/freeze [:x :z] [:x :y :z]) (x/wrap q))
       #(do (hook+ % :fixed-update :ref/height #'height-step)
            (state+ % :ref/height [4 4 4 4 4])
            (full-watch %)))]))

;; YAW

(defn yaw-step [^GameObject gob k]
  (let [rt (.transform (state gob :ref)) 
        y (arc -180 (.. gob transform rotation eulerAngles y))
        a (.. rt localRotation eulerAngles)
        t (or (state gob :time/yaw) 0)]
    (if (> (- Time/time t) 5)
      (let [delta 15
            slices (state gob k)
            n (count slices)
            _ (assert (odd? n) "reference for step must be odd")
            h (quot n 2)
            i (+ h (max (- h) (min h (Math/Round (/ y delta) 0))))
            ; n (dec (count slices))
            ; _ (assert (odd? n) "reference for yaw must be even")
            ; h (inc (quot n 2))
            ; i (+ h (max (- h) (min (dec h) (Math/Round (/ y delta) 0))))
            slices (assoc slices i 0)
            i (wrand slices)
            c (dec (slices i))
            r (* delta (- i h))]
        (state+ gob :time/yaw Time/time)
        (set! (.localRotation rt) (euler (v3 (.x a) r (.z a))))
        (update-state gob k assoc i (if (zero? c) 1 c))))))

(defn pitch [q] ;@tcc
  (x/doto (gangorra (x/wrap q))
    #(do (hook+ % :fixed-update :ref/pitch #'pitch-step)
         (state+ % :ref/pitch (vec (repeat 8 8)))
         (full-watch %))))

(defn yaw [q] ;@tcc
  (x/doto (x/wrap q)
    #(do (hook+ % :fixed-update :ref/yaw #'yaw-step)
         (state+ % :ref/yaw (vec (repeat 8 8)))
         (set! (.axis (cmpt+ % HingeJoint)) up)
         (full-watch %))))

(comment todo
  camera change on keyboard
  particle draw on keyboard (trail))
  
(defn kick [^GameObject gob k]
  (.AddForceAtPosition 
    (cmpt gob Rigidbody) 
    (v3* up -2.0)
    (v3 0 0 0.325)))  
  
(defn grelay [[d eps] position]
  (as-> quad-4 q
        (x/complete 
          #(do (set! (.. % transform position) position)
               (hook+ % :fixed-update :ctrl/pitch #'pitch-ctrl)
               (state+ % :ctrl/pitch (new-relay d (- d) eps 1))
               (state+ % :ctrl/offset [0.2 0.0 0.2 0.0]))
          q)
        (x/doto q #(hook+ % :on-enable :initial-kick #'kick))
        (first (render (gangorra q)))
        (delay (spit-watch q
                 {:title (str "Relé " (format "%.3f" (* d 100)) "% " eps "°")}))))

(defn pitch-relay []
  (render background) ;todo: check if controller gains are from % to °
  (mapv grelay [[0.02 3] [0.02 2] [0.02 1] [0.02 0]]
        (for [i (range 4)] (v3 i 0.5 0))))