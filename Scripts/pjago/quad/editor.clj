(do (in-ns 'pjago.quad)
    (import '[UnityEngine GameObject Application Input KeyCode Canvas Screen GUI Color Rect ParticleSystem TrailRenderer]
            '[UnityEngine.UI Button Toggle Text Scrollbar]
            '[UnityEngine.SceneManagement SceneManager]
            '[UnityEngine.Events UnityAction |UnityAction`1|]
            '[UnityEngine.EventSystems EventSystem]))

;; HOOKS

(def new-rect (memoize #(Rect. %1 %2 %3 %4)))

(defn label [[x y w h] txt]
  (GUI/Label (new-rect x y w h) txt))

(defn aero-info [^GameObject gob k]
  ; (when (= (count (objects-tagged (str ::quad))) 1))
    (let [{:keys [thrust drag spin speed]} (state gob)
          H (/ Screen/height 4)
          W (/ Screen/width 3)]
      (set! GUI/color Color/black)
      ; (if-cmpt (object-tagged ":camera-main") [cam Camera]
      ;   (set! GUI/color Color/white)
      ;   (set! GUI/color Color/black))
      ; (if (state (object-tagged (str ::canvas)) :paused)
      ;   (set! GUI/color Color/black))
      (label [0 0 W H] (format "P: %.2f" (float (* thrust gf:N))))
      (label [0 15 W H] (format "T: %.2f" (float drag))) ;N.m
      (label [0 30 W H] (format "w: %.2f" (float spin)))
      (label [0 45 W H] (format "v: %.2f" (float speed)))))

(defn spin-info [^GameObject gob k]
  ; (when (= (count (objects-tagged (str ::quad))) 1)
    (let [props (state gob :props)
          input (state gob :ctrl/input)
          n (count props)
          H (/ Screen/height 4)
          W (- Screen/width 70)]
      (set! GUI/color Color/black)
      ; (if-cmpt (object-tagged ":camera-main") [cam Camera]
      ;   (set! GUI/color Color/white)
      ;   (set! GUI/color Color/black))
      ; (if (state (object-tagged (str ::canvas)) :paused)
      ;   (set! GUI/color Color/black))
      (dotimes [i n]
        (let [u (- (state (nth props i) :spin/set) (nth input i))]
          (label [W (- Screen/height 20.75 (* (- 3 i) 15)) W H]
            (format "u(%d): %.4f" i (float u)))))))

(defn ctrl-info [^GameObject gob k]
  ; (when (= (count (objects-tagged (str ::quad))) 1)
    (let [{:keys [props]} (state gob)
          H (/ Screen/height 4)
          W (- Screen/width 79)]
      (set! GUI/color Color/black)
      (dotimes [i (count props)]
        (as/let [p (nth props i) {:keys [thrust drag]} (state p)]
          (label [W (* i 15) W H]
            (format "P(%d): %+2.2f" i (float (* thrust gf:N))))
          (label [0 (* i 15) W H]
            (format "T(%d): %+.4f" i (float drag)))))))

(defn state-info [^GameObject gob k]
  ; (when (= (count (objects-tagged (str ::quad))) 1)
    (as/let [H (/ Screen/height 4)
             W (/ Screen/width 3)
             y (- Screen/height 20.75)]
      (as/let [e (.. gob transform rotation eulerAngles)
               h (.. gob transform position y)]
        (label [0 (- y 0) W H] (format "pitch: %.2f" (arc -180 (.-x e))))
        (label [0 (- y 15) W H] (format "yaw: %.2f" (arc -180 (.-y e))))
        (label [0 (- y 30) W H] (format "roll: %.2f" (arc -180 (.-z e))))
        (label [0 (- y 45) W H] (format "height: %.2f" h)))))

;todo: cycle info, which has all the info but integrates with cycle-gui
(defn cycle-info [^GameObject gob k]
  (let []))

;todo: thrusts, speeds, drags, eulers + height
(defn cycle-gui [^GameObject gob]
  (let [ps (state gob :props)
        on #(cmpt % (hook-types :on-gui))
        pe (some #(if (.enabled (on %)) %) ps)]
    (run! #(set! (.enabled (on %)) false) ps)
    (loop [ps (cycle ps)]
      (if (or (nil? pe) (= (first ps) pe))
        (doto (second ps) (-> on .enabled (set! true)))
        (recur (rest ps))))))

;; HELPERS

(defn get-player
  ([] (get-player (object-tagged (str ::canvas))))
  ([gob]
   (or (obj-nil (state gob :player))
       (object-tagged (str ::quad)))))

(defn switch-ctrl [ctrl-map]
  (doseq [[k v] ctrl-map]
    (let [k (keyword "ctrl" (name k))
          q (get-player)
          c (cond 
              (sequential? v)
              (apply (first v) (rest v))
              (fn? v)
              (v))]
      (update-state q :canvas/ctrl assoc k c)
      (state+ q k c))))

(defmacro >> [id & fields] ;TODO: rethink the name?
  (let [ff (first fields)
        rf (rest fields)
        id (cond 
             (keyword? id) `(object-tagged ~(str id)) 
             ; (symbol? id)
             ; (if-let [v (resolve id)]
             ;   (if (and (var? v) (fn? @v))
             ;     `(object-tagged ~(str (first (v))))
             ;     id)
             ;   `(object-tagged ~(str *ns* "/" id)))
             :default id)
        ff (cond 
             (symbol? ff)
             (if-let [v (resolve ff)]
               (if (isa? v UnityEngine.Component)
                 `(cmpt ~id ~v) 
                 `(~ff ~id)) 
               `(~ff ~id))
             (keyword? ff) `(state ~id ~ff)
             (list? ff) `(-> ~id ~ff)
             :default `(~ff ~id))]
    (if (seq fields) 
      `(-> ~ff ~@rf)
      id)))

;; CANVAS

(def default-ctrl 
  #:ctrl
  {:euler (new-euler map->Euler2Ctrl)
   :height (lead-dh)
   :pitch (lead-d)
   :roll (lead-d)
   :yaw (lead-dy)})

(def default-canvas
  (merge
    ^{::x/init ::canvas} {:time {:state 0} :watches {:state #{::quad}}}
    (zipmap [:paused :relay :step :pid]
            (repeat {:state false}))
    (zipmap [:euler :height :pitch :yaw :roll] ;@bug can't have height false at first (whyyy) 
            (repeat {:state true}))
    {:toggle/skip {:state true}
     :time-scale {:state 1.0}
     :right-input {:state 0.05}
     :forward-input {:state 0.05}
     :vertical-input {:state 0.10}
     :horizontal-input {:state 0.10}
     :player/pitch {:state 0.5}
     :player/roll {:state 0.5}
     :player/yaw {:state 0.5}
     :scrollbar/key {:state :time-scale}
     :scrollbar/set {:state [:time-scale
                             :right-input
                             :forward-input
                             :horizontal-input
                             :vertical-input
                             :player/roll
                             :player/pitch
                             :player/yaw]}}))

(def toggle-tree
  (-> (make-hierarchy)
      (derive :height ::constraint)
      (derive :roll ::constraint)
      (derive :pitch ::constraint)
      (derive :yaw ::constraint)
      (derive ::constraint ::refresh)
      (derive :step ::refresh)
      (derive :relay ::refresh)))

(defmulti canvas-toggle (fn [gob k] k) :hierarchy #'toggle-tree)

(def cw (comp (map str) (mapcat objects-tagged)))

;chain futures and log the result @todo
(defn canvas-spit-watch 
  ([] (canvas-spit-watch (object-tagged (str ::canvas))))
  ([gob]
   (let [togg (select-keys (state gob) (keys (state gob :toggles)))
         info {:dt Time/fixedDeltaTime :canvas-state togg}]
     (into []
       (comp cw (map #(spit-watch % info)))
       (state gob :watches)))))

(defn canvas-pause [gob]
  (let [watches (into [] cw (state gob :watches))]
    (doseq [fxup (map #(cmpt % (:fixed-update hook-types)) watches)]
      (set! (.enabled fxup) false))
    (state+ gob :paused true)))

(defn canvas-resume [gob]
  (let [watches (into [] cw (state gob :watches))]
    (doseq [fxup (map #(cmpt % (:fixed-update hook-types)) watches)]
      (set! (.enabled fxup) true))
    (state+ gob :paused false)))

;situation: I kept confusing quad with gob here, so I was
;adding a Rigidbody to the canvas, and couldn't click anything
;lesson learned: perhaps if instead of accepting something w/o
;a Rigidbody and then adding it, maybe I should break.
;this way I could easily detect the source of the problem.
;there are props and cons, I have to think about it

;trying to reset state is a good test to identify complexity @wip
;I can always re-render a new gobj, but I would like to keep any user state
(defn canvas-restart
  ([gob] (canvas-restart gob (get-player gob)))
  ([gob q]
   (let [txt (state gob :time/txt)]
     (if (obj-nil q)
       (let [{:ctrl/keys [height yaw pitch roll euler]} (state q)
             fx (cmpt q (:fixed-update hook-types))
             rb (cmpt q Rigidbody)
             tr (objects-tagged (str ::trail))]
         (when height (reset height))
         (when yaw (reset yaw))
         (when roll (reset roll))
         (when pitch (reset pitch))
         (if (and euler (instance? Euler2Ctrl euler))
            (let [rp (.. euler relay-p) ;@wip (this is a bad idea)
                  ry (.. euler relay-y)
                  rr (.. euler relay-r)]
              (set! (.u rp) (conj (.uo rp) (.sup rp)))
              (set! (.u ry) (conj (.uo ry) (.sup ry)))
              (set! (.u rr) (conj (.uo rr) (.sup rr)))))
         (doseq [[p o] (partition 2 (interleave (state q :props) (state q :ctrl/offset)))]
           (update-state p :spin/fifo #(into % (repeat (count %) o))) ;@cheat
           (state+ p :spin/set o))
         (kinematico q :kinematic)
         (dynamico q :dynamic)
         (when-let [rf (state q :ref)]
           (set! (.. rf transform localRotation) (qt))
           (set! (.. rf transform localPosition) (v3 0.0 -0.1 0.0)))
         (set! (.velocity rb) (v3 0))
         (set! (.angularVelocity rb) (v3 0))
         (set! (.. q transform position) (v3 0))
         (set! (.. q transform rotation) (qt))
         (state+ gob :player/pitch 0.5)
         (state+ gob :player/roll 0.5)
         (state+ gob :player/yaw 0.5)
         (run! #(.Clear (cmpt % TrailRenderer)) tr)
         (canvas-toggle gob ::constraint)))
     (doseq [cam (state gob :camera)]
       (set! (.. cam transform position) (state cam :focus)))
     (set! (.text txt) (format "%.2f" 0.0))
     (state+ gob :time Time/time))))

;print the address on screen @todo
(defn canvas-save [gob]
  (canvas-spit-watch gob))

(defn canvas-quit [gob]
  ;(Application/Quit)) ; Application/Quit crashes build
  (if-not Application/isEditor
    (.Kill (System.Diagnostics.Process/GetCurrentProcess))))

(defn canvas-slide [gob value]
  (if (state gob :paused)
    (let [ts (Math/Round (m/lmap value 0.0 1.0 0.01 1.0) 2)
          sc (state gob :scrollbar/value-txt)
          sk (state gob :scrollbar/key)]
      (set! (.text sc) (format "%.2f" ts))
      (state+ gob sk ts)
      (if-let [q (and (= (namespace sk) "player") (obj-nil (state gob :player)))]
        (let [x (m/lmap (state gob :player/pitch) 0.01 1.0 176.4 -180)
              z (m/lmap (state gob :player/roll) 0.01 1.0 176.4 -180)
              y (m/lmap (state gob :player/yaw) 0.01 1.0 176.4 -180)]
          (set! (.. q transform rotation) (euler (v3 x y z))))))
    (if-cmpt (first (state gob :camera)) [cam Camera]
      (set! (.orthographicSize cam) 
            (m/lmap (* value value) 0.0 1.0 0.3 2.1)))))

;basically this should remove some hooks, enable others
(defmethod canvas-toggle :pid [gob k]
  (if (state gob k)
    (switch-ctrl {:roll i-pd :pitch i-pd :yaw i-pdy :height i-pdh})
    (switch-ctrl {:roll lead-d :pitch lead-d :yaw lead-dy :height lead-dh})))

; it is better to have step, relay, wind as buttons, and add the home button
(defmethod canvas-toggle :step [gob k]
  (when-let [q (get-player gob)]
    (hook- q :fixed-update :ref/height)
    (hook- q :fixed-update :ref/roll)
    (hook- q :fixed-update :ref/pitch)
    (hook- q :fixed-update :ref/yaw)
    (when (state gob k)
      (when (state gob :height)
        (hook+ q :fixed-update :ref/height #'height-step)
        (state+ q :ref/height [4 4 4 4 4 4 4]))
      (when (state gob :roll)
        (hook+ q :fixed-update :ref/roll #'roll-step)
        (state+ q :ref/roll (vec (repeat 11 4))))
      (when (state gob :pitch)
        (hook+ q :fixed-update :ref/pitch #'pitch-step)
        (state+ q :ref/pitch (vec (repeat 11 4))))
      (when (state gob :yaw)
        (hook+ q :fixed-update :ref/yaw #'yaw-step)
        (state+ q :ref/yaw (vec (repeat 13 4))))
      (((x/execution-order quad-ex) (x/fx)) q))))

(defmethod canvas-toggle :relay [gob k]
  (when-let [q (get-player gob)]
    (let [{:ctrl/keys [height yaw roll pitch]} (state q :canvas/ctrl)]
      (state+ q :ctrl/height height)
      (state+ q :ctrl/yaw yaw)
      (state+ q :ctrl/roll roll)
      (state+ q :ctrl/pitch pitch))
    (if (state gob k)
      (let [{:ctrl/keys [height yaw roll pitch]} (state gob)]
        (when (state gob :height)
          (if (and height (not (instance? height RelayCtrl))) 
            (update-state q :canvas/ctrl assoc :ctrl/height height))
          (state+ q :ctrl/height (new-relay 0.02 -0.02 0 1)))
        (when (state gob :yaw)
          (if (and yaw (not (instance? yaw RelayCtrl))) 
            (update-state q :canvas/ctrl assoc :ctrl/yaw yaw))
          (state+ q :ctrl/yaw (new-relay 0.02 -0.02 0 1)))
        (when (state gob :roll)
          (if (and roll (not (instance? roll RelayCtrl))) 
            (update-state q :canvas/ctrl assoc :ctrl/roll roll))
          (state+ q :ctrl/roll (new-relay 0.02 -0.02 0 1)))
        (when (state gob :pitch)
          (if (and pitch (not (instance? pitch RelayCtrl))) 
            (update-state q :canvas/ctrl assoc :ctrl/pitch pitch))
          (state+ q :ctrl/pitch (new-relay 0.02 -0.02 0 1)))))))
      

(def hj-height (v3 0.0 0.057612861399970 0.0)) ;@tcc

;this is a everything check, for performance toggle it incrementally @todo
(defmethod canvas-toggle ::constraint [gob k]
  (canvas-toggle gob :relay) ;@feature?
  (canvas-toggle gob :step)
  (if-let [q (get-player gob)]
    (let [{:keys [height roll pitch yaw]} (state gob)
          ck (keyword "ctrl" (name k))
          h (if height 1 0)
          p (if pitch 1 0)
          r (if roll 1 0)
          y (if yaw 1 0)
          mask (+ (* 8 h) (* 4 r) (* 2 p) y) ;store state as a bitmask @todo
          rb (ensure-cmpt q Rigidbody)
          lup (hook-types :late-update)
          lca (keep #(cmpt % lup) (state gob :camera))]
      (set! (.constraints rb) RigidbodyConstraints/None)
      (destroy-immediate (cmpt q HingeJoint)) ;avoids race condition
      (compute-tensor q)
      (off-zero q :ctrl/offset)
      (doseq [ck (keys default-ctrl) :let [k (keyword (name ck))]]
        (when-let [ctrl (state q ck)] ;this is messy @clean
          (set! (.enabled ctrl) (state gob k))
          (set! (.enabled ((state q :canvas/ctrl) ck)) (state gob k))))
      (run! #(set! (.enabled %) true) lca)
      (cond
        (= mask 4)
        (with-cmpt q [hj HingeJoint]
          (set! (.axis hj) forward)
          (run! #(do (set! (.enabled %) false)
                     (follow-pos (gobj %) :follow)) ;otherwise it misses 1
                lca)
          (set! (.anchor hj) 
            (v3map / 
              (v3+ hj-height (.centerOfMass rb))
              (.. q transform lossyScale)))
          (set! (.autoConfigureConnectedAnchor hj) false)
          (update-state q :ctrl/offset assoc 0 0.0)
          (update-state q :ctrl/offset assoc 2 0.0))
        (= mask 2)
        (with-cmpt q [hj HingeJoint]
          (set! (.axis hj) right)
          (run! #(do (set! (.enabled %) false)
                     (follow-pos (gobj %) :follow)) ;otherwise it misses 1
                lca)
          (set! (.anchor hj)
            (v3map / 
              (v3+ hj-height (.centerOfMass rb))
              (.. q transform lossyScale)))
          (set! (.autoConfigureConnectedAnchor hj) false)
          (update-state q :ctrl/offset assoc 1 0.0)
          (update-state q :ctrl/offset assoc 3 0.0))
        (= mask 0)
        (set! (.constraints rb) RigidbodyConstraints/FreezeAll)
        :else
        (let [cpos (if (zero? (bit-and mask 8)) [:y] [])
              crot (cond-> []
                     (zero? (bit-and mask 4)) (conj :z)
                     (zero? (bit-and mask 2)) (conj :x)
                     (zero? (bit-and mask 1)) (conj :y))]
          (((x/freeze cpos crot) (x/fx)) q))))))

(defn switch-player 
  ([p] (switch-player (object-tagged (str ::canvas)) p))
  ([gob now]
   (let [tgg (keys (state gob :toggles))
         pre (state gob :player)]
     (state+ gob :player now)
     (when (obj-nil pre)
       (state+ pre :canvas/toggle (select-keys (state gob) tgg))
       (with-cmpt pre [on-gui (hook-types :on-gui)]
         (set! (.enabled on-gui) false)))
     (when (obj-nil now)
       (reduce-kv state+ gob (state now :canvas/toggle))
       (let [fixed (some? (cmpt now HingeJoint))] ;use mask state instead @todo 
         (doseq [cam (state gob :camera)]
           (set! (.enabled (cmpt cam (hook-types :late-update))) (not fixed))
           (mario-cam! cam now)))
       (with-cmpt now [on-gui (hook-types :on-gui)]
         (set! (.enabled on-gui) true))))))

(defn canvas-update [gob _] ;probably the worst code I've ever made. nice!
  (let [bar (state gob :scrollbar)
        tgg (state gob :toggles)
        esc (state gob :paused)
        pan (state gob :panel)
        old (state gob :time)
        txt (state gob :time/txt)
        sca (state gob :scrollbar/value-txt)
        now Time/time]
    (if-let [q (obj-nil (state gob :player))]
      (let [uy (* (state gob :horizontal-input) (Input/GetAxis "Horizontal"))
            uh (* (state gob :vertical-input) (Input/GetAxis "Vertical"))
            ux (* (state gob :forward-input) (Input/GetAxis "Forward"))
            uz (* (state gob :right-input) (Input/GetAxis "Right"))]
        (if (or (Input/GetKey KeyCode/RightControl) (Input/GetKey KeyCode/LeftControl))
          (do (state+ q :ref/input [(* m/deg:rad ux) (* m/deg:rad uy) (* m/deg:rad uz) uh])
              (state+ q :ctrl/input [0.0 0.0 0.0 0.0]))
              ;(state+ q :disturbance [0.0 0.0 0.0 0.0]))
          (do ;(state+ q :disturbance [ux uy uz uh])
              (state+ q :ctrl/input
                [(+ (/ uh 4) (/ ux +2) (/ uy +4))
                 (+ (/ uh 4) (/ uz +2) (/ uy -4)) 
                 (+ (/ uh 4) (/ ux -2) (/ uy +4))
                 (+ (/ uh 4) (/ uz -2) (/ uy -4))])
              (state+ q :ref/input [0.0 0.0 0.0 0.0])))))
    (when (state gob :toggle/skip)
      (doseq [[k v] (select-keys (state gob) (keys tgg))
              :let [toggle (get tgg k)]]
        (when-not (= (.isOn toggle) v) ;bad react vibes
          (set! (.isOn toggle) v)))
      (state+ gob :toggle/skip true))
    (when-not (state gob :toggle/skip)
      (canvas-toggle gob ::constraint) ;will check everything @bug
      (state+ gob :toggle/skip true))
    (when-not (= (.activeSelf pan) esc)
      (if esc
        (do (if-let [q (obj-nil (state gob :player))]
              (let [e (v3map arc (v3 -180) (.. q transform rotation eulerAngles))]
                (state+ gob :player/pitch (m/lmap (.x e) 176.4 -180 0.01 1.0))
                (state+ gob :player/roll (m/lmap (.z e) 176.4 -180 0.01 1.0))
                (state+ gob :player/yaw (m/lmap (.y e) 176.4 -180 0.01 1.0))))
            (let [k (state gob :scrollbar/key)
                  v (state gob :scrollbar/value-txt)
                  x (-> (state gob k)
                        (m/constrain 0.01 1.0)
                        (m/lmap 0.01 1.0 0.0 1.0)
                        float)]
              (state+ gob :scrollbar/skip true)
              (set! (.value bar) x)
              (set! Time/timeScale 0.0)
              (if (= (namespace k) "player")
                (set! (.text v) (format "%.4f" x)))
              (set! (.text txt) (format "%.2f" (- now old)))
              (canvas-pause gob)))
        (let [cam (cmpt (first (state gob :camera)) Camera)
              x (m/constrain (.orthographicSize cam) 0.3 2.1)]
          (set! (.value bar) (Mathf/Sqrt (m/lmap x 0.3 2.1 0.0 1.0)))
          (set! Time/timeScale (state gob :time-scale))
          (canvas-resume gob)))
      (.SetActive pan esc))
    (if (state gob :scrollbar/skip) 
      (state+ gob :scrollbar/skip false))
    (if (Input/GetKeyDown KeyCode/Escape)
      (if-not (or (Input/GetKey KeyCode/LeftShift) (Input/GetKey KeyCode/RightShift))
        (state+ gob :paused (not esc))
        (let [show-ui (not (state gob :hide-ui))] 
          (state+ gob :hide-ui show-ui)
          (if-cmpt (get-player) [og (hook-types :on-gui)]
            (set! (.enabled og) show-ui))
          (doseq [child (rest (children gob))]
            (.SetActive child show-ui)))))
    (if (Input/GetKeyDown KeyCode/Tab)
      (if esc
        (let [rev (or (Input/GetKey KeyCode/LeftShift) (Input/GetKey KeyCode/RightShift))
              all (vec (cond-> (state gob :scrollbar/set) rev (-> vec rseq)))
              pre (state gob :scrollbar/key)
              sck (second (drop-while #(not= % pre) (conj all (first all))))
              sca (state gob :scrollbar/txt)
              scb (state gob :scrollbar/value-txt)
              scv (m/constrain (state gob sck) 0.01 1.0)]
          (state+ gob sck scv)
          (state+ gob :scrollbar/key sck)
          (set! (.text sca) (str (name sck) ":"))
          (if (= (namespace sck) "player")
            (set! (.text scb) (format "%.4f" scv))
            (set! (.text scb) (format "%.2f" scv)))
          (state+ gob :scrollbar/skip true)
          (set! (.value bar) (float (m/lmap scv 0.01 1.0 0.0 1.0))))
        (if (or (Input/GetKey KeyCode/LeftShift) (Input/GetKey KeyCode/RightShift))
          (let [all (vec (objects-tagged (str ::quad))) ;always sorted
                pre (get-player gob)
                now (second (drop-while #(not= % pre) (conj all (first all))))]
            (switch-player gob now)
            (state+ gob :toggle/skip true))
          (let [[c1 c2] (state gob :camera)
                cam (cmpt c2 Camera)
                x (m/constrain (.orthographicSize cam) 0.3 2.1)]
            (state+ gob :camera [c2 c1])
            (set! (.value bar) (Mathf/Sqrt (m/lmap x 0.3 2.1 0.0 1.0)))
            (mario-cam! c2 (get-player))
            (.SetActive c1 false)
            (.SetActive c2 true)))))))
    ; (if-not (.IsPointerOverGameObject EventSystem/current) ;@bug
    ;   (let [scroll (Input/GetAxis "Mouse ScrollWheel")]
    ;     (if-not (zero? scroll)
    ;       (let [value (m/constrain (- (.value bar) scroll) 0.0 1.0)]
    ;         (set! (.value bar) (float value))
    ;         (canvas-slide gob value)))))))

(defn canvas
  ([] [::canvas])
  ([gob child])
  ([gob]
   (let [[pan pan1 pan2 pan3] (mapv gobj (children gob))
         bar (cmpt (.. pan3 transform (Find "bar")) Scrollbar)
         rst (mapcat children [pan1 pan2])
         chk (map (comp keyword #(.name %)) rst)
         tgg (map #(cmpt % Toggle) rst)
         can (cmpt gob Canvas)
         txt (cmpt (.. pan transform (Find "time")) Text) ;bad om.next vibes
         sca (cmpt (.. pan transform (Find "input")) Text)
         scb (cmpt (.. pan transform (Find "scale")) Text)
         save (cmpt (.. pan transform (Find "save")) Button)
         restart (cmpt (.. pan transform (Find "restart")) Button)
         quit (cmpt (.. pan transform (Find "quit")) Button)]
     (.AddListener (.onValueChanged bar) 
       #(if-not (state gob :scrollbar/skip) 
          (canvas-slide gob %)))
     (.AddListener (.onClick save) #(canvas-save gob))
     (.AddListener (.onClick restart) #(canvas-restart gob))
     (.AddListener (.onClick quit) #(canvas-quit gob))
     (doseq [[kw toggle] (zipmap chk tgg)]
       (set! (.isOn toggle) false)
       (.AddListener (.onValueChanged toggle)
         (if (isa? toggle-tree kw ::refresh)
           (fn [_] 
             (state+ gob kw (.isOn toggle))
             (state+ gob :toggle/skip false))
           (fn [_] 
             (state+ gob kw (.isOn toggle))
             (canvas-toggle gob kw)))))
     ;this roles+ here is something I was trying to avoid
     ;because getting from maps that don't hold imutable state is messy
     ;I "solved" it for {} inits by commmon.processing/fresh-state
     ;in this case there is no mutable stable, so it's fine @bug
     (roles+ gob (merge (zipmap chk (repeat {:state false})) default-canvas))
     (state+ gob :toggles (zipmap chk tgg))
     (state+ gob :scrollbar bar)
     (state+ gob :panel pan)
     (state+ gob :time/txt txt)
     (state+ gob :scrollbar/txt sca) ;components would help
     (state+ gob :scrollbar/value-txt scb)
     (hook+ gob :update #'canvas-update)
     (.SetActive pan false))))

(defn canvas-scene
  ([] [nil #'canvas ::event-system last-camera other-camera height-scene])
  ([graph child])
  ([graph]
   (let [{:keys [::canvas :last-camera :other-camera ::x/scene]} 
         (x/by-tag (children graph) first)
         wind (->> (children scene) (group-by x/tag)
                   ::wind-zone first)]
     (state+ canvas :wind-zone wind)
     (state+ canvas :camera [last-camera other-camera])
     (.SetActive wind false)
     (.SetActive other-camera false))))

;; SCENE

; if the sim is on a super computer, connected with multiple repls
; then the learning could be interactive, and examples could be shared easily
; this is for a future release, where each student will have a namespace
; objects on different namespaces won't interact, but they will be visible
; slow notebooks will watch a stream from the super computer (haha)
; the professor will have access to student's data and could save it
; a remote control from MATLAB could also be implemented
; live plots can also be implemented

; (defn clear ; too desctrutive, maybe later after I sort namespaces out
;   ([] (clear :except [#(not= (namespace (x/tag %)) (str *ns*))]))
;   ([& {:keys [except]}]
;    (let [all (new |System.Collections.Generic.List`1[UnityEngine.GameObject]| (.rootCount (SceneManager/GetActiveScene)))
;          fns (filter fn? except)]
;      (.GetRootGameObjects (SceneManager/GetActiveScene) all)
;      (if (some some? fns)
;        (run! destroy (remove (apply some-fn fns) all))
;        (run! destroy all)))))

(defn clear [tag]
  (if-let [obj (obj-nil tag)]
    (destroy obj)
    (destroy (object-tagged (str tag)))))

(defn clear-all [tag & {pred :except}]
  (if pred
    (run! destroy (remove pred (objects-tagged (str tag))))
    (run! destroy (objects-tagged (str tag)))))

(def player
  (x/doto (x/wrap qboth)
    #(when-let [c (object-tagged (str ::canvas))]
       (let [toggles (keys (state c :toggles))
             ctrls (keys default-ctrl)]
         (state+ % :canvas/toggle (select-keys (state c) toggles))
         (state+ % :canvas/ctrl (select-keys (state %) ctrls)))
       (switch-player c %)
       (canvas-restart c %))))

;this code is ugly, it can be better than this @todo
(defn relay-3
  ([] (relay-3 {}))
  ([{:keys [space height yaw pitch roll]
      :or {space 1 height [0.01 0.1] yaw [0.02 1] pitch [0.02 1] roll [0.02 1]}}]
   (switch-player nil)
   (when-let [c (object-tagged (str ::canvas))]
     (let [qmid (volatile! nil)]
       (reduce-kv state+ c (zipmap (keys (state c :toggles)) (repeat false)))
       (state+ c :relay true)
       (when pitch
         (state+ c :pitch true)
         (let [x (- space) z (- x) peps (map * [0 1 2] (repeat (second pitch)))]
           (doseq [[y eps] (map list [-0.5 0.0 0.5] peps)
                   :let [_ (state+ c :player nil) ;@bug @switch-user
                         q (first (render player))]]
             (state+ q :ctrl/pitch (new-relay (first pitch) eps 1))
             (let [hj (cmpt q HingeJoint) ca (.connectedAnchor hj)]
               (set! (.connectedAnchor hj) (v3+ ca (v3 x y z))))
             (set! (.. q transform position) (v3 x y z))
             (set! (.enabled (cmpt q (hook-types :on-gui))) false))))
       (when yaw
         (state+ c :pitch false)
         (state+ c :yaw true)
         (let [x 0 z (- x) yeps (map * [0 1 2] (repeat (second yaw)))]
           (doseq [[y eps] (map list [-0.5 0.0 0.5] yeps)
                   :let [_ (state+ c :player nil) ;@bug @switch-user
                         q (first (render player))]]
             (if (zero? y) (vreset! qmid q))
             (state+ q :ctrl/yaw (new-relay (first yaw) eps 1))
             (set! (.. q transform position) (v3 x y z))
             (set! (.enabled (cmpt q (hook-types :on-gui))) false))))
       (when roll
         (state+ c :yaw false)
         (state+ c :roll true)
         (let [x space z (- x) reps (map * [0 1 2] (repeat (second roll)))]
           (doseq [[y eps] (map list [-0.5 0 0.5] reps)
                   :let [_ (state+ c :player nil) ;@bug @switch-user
                         q (first (render player))]]
             (state+ q :ctrl/roll (new-relay (first roll) eps 1))
             (let [hj (cmpt q HingeJoint) ca (.connectedAnchor hj)]
               (set! (.connectedAnchor hj) (v3+ ca (v3 x y z))))
             (set! (.. q transform position) (v3 x y z))
             (set! (.enabled (cmpt q (hook-types :on-gui))) false))))
       (when height
         (state+ c :roll false)
         (state+ c :height true)
         (let [heps (map * [0 1 2] (repeat (second height)))]
           (doseq [[x z eps] (map conj [[1 3] [0 0] [-1 -3]] heps) 
                   :let [_ (state+ c :player nil) ;@bug @switch-user
                         q (first (render player))]]
             (-> q (state :ref) (.. transform localPosition) (set! (v3 0 3 0)))
             (state+ q :ctrl/height (new-relay (first height) eps 1))
             (set! (.. q transform position) (v3 x -3 z))
             (set! (.enabled (cmpt q (hook-types :on-gui))) false))))
       (state+ c :player nil)
       (switch-player c @qmid)
       (set! (.enabled (cmpt @qmid (hook-types :on-gui))) false)))))

(defn relay-3e
  ([] (relay-3e {}))
  ([args] (relay-3 (merge args {:space 0.5 :height false}))))

(defn main [& args]
  (render background)
  (render canvas-scene)
  (render player)
  (if-let [manager (obj-nil (first args))]
    (destroy manager)))

;; PLAYGROUND
(comment
    (def q (>> ::quad))
    (do (set! (>> q Rigidbody .mass) (float 0.4))
        (compute-tensor q)
        (set! (>> q Rigidbody .centerOfMass) (v3 0 -0.0065 0)) ;resonable (but lacks)
        (set! (>> q Rigidbody .mass) (float 1.2))
        (hook- q :fixed-update ::carry))
    (do (set! (>> q Rigidbody .centerOfMass) (v3 0 0.0866667 0)) ;unresonable
        (set! (>> q Rigidbody .mass) (float 0.6))
        (hook+ q :fixed-update ::carry #'weight-carry))
    (do (set! (>> q Rigidbody .centerOfMass) (v3 0 -0.0065 0)) ;midway
        (set! (>> q Rigidbody .mass) (float 0.6))
        (hook+ q :fixed-update ::carry #'weight-carry)))