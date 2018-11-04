(ns pjago.quad
  (:use arcadia.core arcadia.linear pjago.hooks pjago.watch)
  (:import [UnityEngine Collision WindZone CapsuleCollider Collider Application GameObject Transform Vector3 Matrix4x4 Quaternion Space ForceMode Time Rigidbody JointMotor Joint HingeJoint JointSpring FixedJoint Material MeshRenderer Camera BoxCollider SphereCollider RigidbodyConstraints Component Rect GUI Physics]
           ArcadiaState ArcadiaBehaviour
           [Csharp M])
  (:require [common.processing :as x]
            [common.math :as m]
            [pjago.repl] ;for side effect
            ; [magic.api :as w]
            [arcadia.sugar :as as]
            [clojure.string :as str]))

;; CONST

(def up (v3 0.0 1 0.0)) ;propeller is facing up
(def right (v3 1 0.0 0.0))
(def forward (v3 0.0 0.0 1))
(def gf:N (/ 1000 (m/abs (.y Physics/gravity))))
(def v (* 11.1 0.5)) ;nominal, center mid point
(def rho 1.2754) ;[kg:m³] CNPT air density
(def kv (/ 1000 m/rpm.s:rad)) ;1000 kv motors
(def u->w #(* kv v (m/sqrt (max 0 %))))
(def umin 0.10)
(def umax 0.90)
(def wmin (u->w umin)) ;esc seems to cut-off?
(def wmax (u->w umax)) ;esc seems to saturate?
(def w->u #(cond ;w->u->w does not exactly roundtrips
             (< % wmin) 0  ;cutoff
             (> % wmax) 90 ;saturate
             :else (/ (* 100 %) (* kv v))))

;; SPIN

(defn spin-0 [^GameObject gob ang axis]
  (with-cmpt gob [tr Transform]
    (set! (. tr -rotation)
      (q* (. tr -rotation) 
          (angle-axis ang axis)))))

(defn spin-1 [^GameObject gob ang axis]
  (with-cmpt gob [tr Transform rb Rigidbody]
    (as/let [(as/o :props [axis]) (state gob :aero)
             prev (.-rotation tr)
             wt (* ang Time/deltaTime)
             next (qq* prev (angle-axis wt axis))]
      (.MoveRotation rb next))))

(defn spin-1h [^GameObject gob ang] ;there is no magic
  (with-cmpt gob [hj HingeJoint]
    (let [jm (JointMotor.)]
      (set! (.-freeSpin jm) true)
      (set! (.-force jm) (.. hj -motor -force))
      (set! (.-targetVelocity jm) (float (* ang m/deg:rad)))
      (set! (.-motor hj) jm))))

(defn spin-2h [^GameObject gob ang]
  (with-cmpt gob [rb Rigidbody hj HingeJoint]
    (let [torque (v3* (.-axis hj) ang)]
      (.AddRelativeTorque rb torque))))

;https://en.wikipedia.org/wiki/Tennis_racket_theorem
;https://physics.stackexchange.com/questions/128471/how-to-simulate-rotational-instability
;todo: unity is really weird about the way it refreshes
;its inertiaTensor. so I probably should add a case here
;for refreshing the inertia tensor after a child is gone, etc
;another caveat is that a child must not have a rigidbody
;and have a collider in order to unity compose its inertia
;so basically it always assumes constant density
;todo: make it play well with constraints

;this solution assumes the angular momentum is the same 
;when looked from either reference frame (world, local)
;it can only be attached to a root game object!
;plus is not correct if angularVelocity changes
(defn missing-physx-udemy [^GameObject gob k]
  (with-cmpt gob [tr Transform rb Rigidbody]
    (case k
      :unstable-rotation
      (let [i (Matrix4x4/Scale (.inertiaTensor rb))
            M (trs (v3 0) (.rotation tr) (v3 1))
            I (m* M i (.inverse M))
            W (-> (m* (.inverse I) i)
                  (.. -transpose 
                      (MultiplyPoint 
                        (.angularVelocity rb))))
            O (* (.magnitude W) Time/fixedDeltaTime m/tau)]
        (.Rotate tr (.normalized W) O Space/World)))))

;explanation: Unity adds torque in a way it shadows
;the relation between the rotation axis
;this way they act idependenly and intuitively
;solving Euler equations for zero torque, there still
;should be a change in angular velocity to preserve
;angular momentum. this fn adds that missing change

;Euler rotation equation : Iw' + w x (Iw) = Torque
;not an easy EDO. that's why unity ignores the second term

;needs to simulate entire physics here
;https://physics.stackexchange.com/questions/128471/how-to-simulate-rotational-instability
;https://mathoverflow.net/questions/81960/the-dzhanibekov-effect-an-exercise-in-mechanics-or-fiction-explain-mathemat
(defn missing-physx [^GameObject gob k] ;@broken
  (with-cmpt gob [tr Transform rb Rigidbody]
    (case k
      :unstable-rotation
      (let [i (Matrix4x4/Scale (.inertiaTensor rb))
            w (.angularVelocity rb)
            M (trs (v3 0) (.rotation tr) (v3 1))
            W (-> (m* (.inverse M) i)
                  (.. -transpose 
                      (MultiplyPoint 
                        (.angularVelocity rb))))
            I (m* M i (.inverse M))
            L (.MultiplyVector I W)]
        (.Rotate tr
          (v3* (.MultiplyVector
                 (.inverse I)
                 (Vector3/Cross (v3- W) L))
               Time/fixedDeltaTime)
          Space/World)))))

;; THRUST

;todo: drag torque reaction 
;todo: blipstream effect
;todo: asymmetric blade effect (pitch restauration)
;todo: gyroscopic effect
;@tcc: selling point for clojure, each propeller is unique
(definterface IAero
  (^double Thrust [^double angular_speed ^double forward_speed])
  (^double Drag [^double angular_speed ^double forward_speed])
  (^double Ct [^double advance_ratio]) ;thrust coefficient
  (^double Cp [^double advance_ratio]) ;drag coefficient
  (^double Ve [^double angular_speed])) ;exit velocity

;broken: uses quad speed, not prop speed
(defn kinematic [^GameObject gob k]
  (if-cmpt (.. gob transform root) [rb Rigidbody]
    (as/let [(as/o :props [velocity]) rb
             M (.. gob -transform -worldToLocalMatrix)]
      (state+ gob k
        (case k
          :speed
          (Vector3/Dot (.MultiplyVector M velocity) up)
          :spin
          (or (state gob :spin/set) 0))))
    (state+ gob k 0)))

;broken: adds force, should leave that to quad-physics
(defn dynamic [^GameObject gob k]
  (if-cmpt (.. gob transform root) [rb Rigidbody]
    (let [{:keys [aero spin speed]} (state gob)]
      (state+ gob k
        (case k
          :thrust
          (doto (.Thrust aero spin speed)
            (->> (v3* up)
                 (.AddRelativeForce rb)))
          :drag
          (doto (.Drag aero spin speed)
            (->> (v3* up)
                 (.AddRelativeTorque rb))))))
    (state+ gob k 0)))

(defn loose-props [^GameObject gob k]
  (if-let [q (parent gob)]
    (doseq [p (state q :props)]
      (when (not= (.. p transform root) (. gob transform))
        (do (update-state q :props (fn [ps] (vec (remove #(= % p) ps))))
            (state+ p :speed 0.0)
            (state+ p :spin 0.0)
            (state+ p :thrust 0.0)
            (state+ p :drag 0.0))))))

;https://physics.stackexchange.com/questions/288382/relative-angular-velocity-of-a-referential-system-translating-and-rotating
;it is funny that this has dynamics as well, but I can't change the name now
;it goes back that Speculation talk from Rich Rickey
(defn kinematico [^GameObject gob k]
  (as/let [(as/with-cmpt tr Transform rb Rigidbody) gob
           (as/o :props [velocity angularVelocity]) rb
           WL (. tr worldToLocalMatrix)
           LW (. tr localToWorldMatrix)]
    (doseq [p (state gob :props)]
      (with-cmpt p [ptr Transform]
        (as/let [r (.MultiplyPoint WL (. ptr position))
                 W (.MultiplyVector LW (.angularVelocity rb))
                 V (v3+ velocity (Vector3/Cross W r))
                 v (.MultiplyVector LW V)]
          (state+ p :speed (Vector3/Dot v up))
          (if-let [fifo (state p :spin/fifo)]
            ;insert 1st order system here!
            ;BIGDOUBT how about if the 1st order system is after the motor
            ;because the time constant of an electric motor should be much
            ;faster than the movement of the air.
            ;NOPE. that's not it. the problem is the lacking of 2x
            ;CONFIRMED! not every inverter are made the same.
            ;My hypothesis that the ESC was dividing the input by 2
            ;was disproven by Antuña Herrero.
            ;DOUBLE NOPE! It makes sense that the inverter has a mid point
            ;otherwize, there should be a gain of 2², then a gain of 1/2
            ;from the velocity increase and the thrust coefficient decrease
            ;perhaps the mistake is not here, I have to apply the real relay again
            ;but one thing is certain: I can simulate the out of sync ESCs here (3rd)
            ;; 1st Attempt: 1st order system after the thrust
            ; (let [uset (state p :spin/set)
            ;       u-1 (first fifo)
            ;       u (+ (* 0.030233540621287 uset) (* 0.969766459378713 u-1))]
            ;   (update-state p :spin/fifo conj u)
            ;   (state+ p :spin (u->w u-1)))
            ;; 2nd Attempt: 1st order system before the thrust (same thing)
            ; (let [wset (m/sqrt (state p :spin/set)) ;no changes
            ;       w-1 (first fifo)
            ;       w (+ (* 0.030233540621287 wset) (* 0.969766459378713 w-1))]
            ;   (update-state p :spin/fifo conj w)
            ;   (state+ p :spin (u->w (* w-1 w-1))))
            ;; 3rd Attempt: 1st order system with random (~fractional) delay 
            ; because the ESC and the control-loop are not in sync, sometimes
            ; a sample will happen right before the ESC update, but other times
            ; it will have to wait almost untill the another sample.
            ; assuming both have the same mean period, there is a 50% chance of wait
            ; so it almost works as a fractional delay of 1/2 a sampling period
            (let [uset (state p :spin/set) ;@tcc
                  u-1 (first fifo)
                  u (+ (* 0.030233540621287 uset) (* 0.969766459378713 u-1))
                  _ (update-state p :spin/fifo conj u)
                  u-0 (first (state p :spin/fifo))] ;if fifo size is 1, that's u
              (if (> (rand) 0.5)
                (state+ p :spin (u->w u-0))
                (state+ p :spin (u->w u-1))))
            (state+ p :spin (u->w (state p :spin/set))))
          (as-> (state p :spin) dyaw
                (* dyaw Time/fixedDeltaTime)
                (* dyaw m/deg:rad)
                (* dyaw (if (.clockwise (state p :aero)) 1 -1))
                (* dyaw 0.05) ; for visual
            (.Rotate ptr (v3 0 dyaw 0))))))))

(defn dynamico [^GameObject gob k]
  (as/let [(as/with-cmpt rb Rigidbody) gob]
    (doseq [p (state gob :props)]
      (as/let [{:keys [aero spin speed]} (state p)]
        (if (pos? spin) ;esc will spin only one way
          (let [T (.Thrust aero spin speed)
                P (.Drag aero spin speed)]
            (state+ p :thrust T)
            (state+ p :drag P))
          (do (state+ p :thrust 0.0)
              (state+ p :drag 0.0)))))))

(defn quad-physx [^GameObject gob k]
  (as/let [(as/with-cmpt rb Rigidbody) gob
           acc-drag (volatile! (v3 0))]
    (doseq [p (state gob :props)]
      (as/let [{:keys [thrust drag]} (state p)
               (as/with-cmpt tr Transform) p
               pos (.-position tr)
               T (v3* (. tr -up) thrust)
               P (v3* (. tr -up) drag)]
        (.AddForceAtPosition rb T pos)
        (vswap! acc-drag v3+ P)))
    (.AddTorque rb @acc-drag)))

;@tcc
(defmutable UIUCstatic ;APC 10x4.7 SL
  [^float diameter ^float pitch ^boolean clockwise]
  IAero
  (Ve [dyn w] ;air exit velocity
    (* w pitch m/m:rad.in))
  (Ct [_ j] ;thurst coeficient
    0.126969463087248)  ;(m/lmap 5550 5417 5715 0.1263 0.1278)
  (Cp [_ j] ;drag coeficient
    0.0510231543624161) ;(m/lmap 5550 5417 5715 0.0508 0.0513)
  (Drag [dyn w v]
    (* 8.4388826e-10 rho (m/pow w 2)
       (m/pow diameter 5)
       (.Cp dyn 0)
       (if clockwise 1 -1)))
  (Thrust [dyn w v]
    (* 1.01266592e-8 rho (m/pow w 2)
       (m/pow diameter 4)
       (.Ct dyn 0))))

; todo: http://m-selig.ae.illinois.edu/props/propDB.html#APC
; https://physics.stackexchange.com/questions/31811/calculate-quadrotor-propeller-torque-due-to-aerodynamic-drag
(defmutable UIUC ;APC 10x4.7 SL 6513 rpm
  [^float diameter ^float pitch ^boolean clockwise]
  IAero ;todo: use diameter and pitch, different rpm op points
  (Ve [dyn w] ;todo: exit profile, # of blades, shape
    (* w pitch m/m:rad.in))
  (Ct [_ j] ;thrust coeficient
    (m/lmap j 0.378 0.755 0.0698 -0.0281)) ;@windmill
  (Cp [_ j] ;drag coeficient
    (if (< j 0.551)
      (m/lmap j 0.378 0.551 0.0422 0.0302)
      (m/lmap j 0.551 0.755 0.0302 0.0106)))
  (Drag [dyn w v]
    (let [va (- (.Ve dyn w) v) ;true air speed
          j (/ va (* w diameter m/m:in))] ;advance ratio
      (* 8.4388826e-10 rho (m/pow w 2)
         (m/pow diameter 5)
         (.Cp dyn j)
         (if clockwise 1 -1))))
  (Thrust [dyn w v]
    (let [va (- (.Ve dyn w) v) ;true air speed
          j (/ va (* w diameter m/m:in))] ;advance ratio
      (* 1.01266592e-8 rho (m/pow w 2)
         (m/pow diameter 4)
         (.Ct dyn j)))))

; http://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html
(defmutable Stables ;@tcc simple, but accurate
  [^float diameter ^float pitch ^boolean clockwise]
  IAero 
  (Ve [dyn w] ;todo: exit profile, # of blades, shape
    (* w pitch m/m:rad.in))
  (Ct [_ j]
    (/ 1.00771633 (m/sqrt (* diameter pitch)) rho))
  (Cp [_ j] ;only a guess, not from Stables
    (/ 1.00771633 (m/sqrt (* diameter pitch)) rho 2))
  (Drag [dyn w v] ; maybe this value is too big?
    (let [va (- (.Ve dyn w) v)] ;true air speed
      (* 1.06538575E-8 va w (/ m/tau) 0.5
         (m/pow diameter 4.5)
         (/ (m/sqrt pitch))
         (if clockwise 1.0 -1.0))))
  (Thrust [dyn w v]
    (let [va (- (.Ve dyn w) v)] ;true air speed
      (* 4.19443207E-7 va w
         (m/pow diameter 3.5) 
         (/ (m/sqrt pitch))))))

;todo: put link to my tcc here
;todo: model the motor saturation correctly, don't let spin have
;a wrong value. for my tcc is fine since I am not claiming anything
;for velocity. I am just talking about the % of actuation spin/set.
;@TODO: measure the tension on the real thing!
;@TODO: repeat relay with new act distribution
;@TODO: somehow measure the speed of full throttle!!!
(defmutable Martins3 ;? 10x4.5
  [^float diameter ^float pitch ^boolean clockwise]
  IAero
  (Ve [dyn w] ;air exit velocity
    (* w pitch m/m:rad.in))
  (Ct [_ j] ;thurst coeficient
    0.128627856016643) ;0.0880540599742135 ;0.0563545983834968; 0.225418392161187 ;0.14419926157
  (Cp [_ j] ;drag coeficient
    0.051689585773345)
  (Drag [dyn w v]
    (* 8.4388826e-10 rho (m/pow w 2)
       (m/pow diameter 5)
       (.Cp dyn 0)
       (if clockwise 1 -1)))
  (Thrust [dyn w v]
    (* 1.01266592e-8 rho (m/pow w 2)
       (m/pow diameter 4)
       (.Ct dyn 0))))

(defmutable Martins0 ;? 10x4.5
  [^float diameter ^float pitch ^boolean clockwise]
  IAero
  (Ve [dyn w] ;air exit velocity
    (* w pitch m/m:rad.in))
  (Ct [_ j] ;thurst coeficient
    0.086281691483864) ;0.075239066087604
  (Cp [_ j] ;drag coeficient
    0.034672620929384) ;0.030235100548814
  (Drag [dyn w v]
    (* 8.4388826e-10 rho (m/pow w 2)
       (m/pow diameter 5)
       (.Cp dyn 0)
       (if clockwise 1 -1)))
  (Thrust [dyn w v]
    (* 1.01266592e-8 rho (m/pow w 2)
       (m/pow diameter 4)
       (.Ct dyn 0))))

;; PID

(defn new-gpid [^double kp ^double ti ^double td ^double ts]
  (let [i (/ ts ti 2)
        d (/ td ts)]
    (mapv #(* % kp) 
          [(+ i d 1) 
           (- i (* d 2) 1) 
           d])))

(defn new-gpdf [^double kp ^double tf ^double td ^double ts]
  (let [f (m/exp (- (/ ts tf)))
        d (/ td tf)]
    (mapv #(* % kp)
      [(+ d 1)
       (- 0 d f)])))

;auto-tune, or some set up to facilitate ICtrl calculations
(defprotocol ITune
  (tune [this]))

;implement PID, Fuzzy, ANN, etc @todo
;base properties of anyone that implements such protocol:
;n, k, r, y, e, u, uo, enabled
(defprotocol ICtrl
  (filt [this r])
  (diff [this])
  (act [this]))

;revert ordering of ring-buffers, so there is no need to rseq
;and no need to (nth x -1), using positive index is good for []
;(nth x -1) means the x before the one was conjed
;(nth x 1) would be the one that was conjed first
;invert this logic, for seq and rseq as well (?) @todo
;think about performance (make ring-b transient) @todo
(defn step-next [ctrl r y]
  (if (.enabled ctrl)
    (if-not (zero? (.k ctrl))
      (do (set! (.k ctrl) (dec (.k ctrl)))
          (nth (.u ctrl) -1)) ;zero order hold
      (-> (as/sets! ctrl
            k (dec (.n ctrl))
            y (conj (.y ctrl) y)
            r (conj (.r ctrl) (filt ctrl r))
            e (conj (.e ctrl) (diff ctrl))
            u (conj (.u ctrl) (act ctrl)))
          (nth -1)))
    (nth (.uo ctrl) -1))) ;landing procedure @todo

(defn set-pid
  [{:keys [kp ti td ts]}]
  {:gpid (new-gpid kp ti td ts)})

(defn set-pi-d
  [{:keys [kp ti td ts]}]
  {:gpi (new-gpid kp ti 0.0 ts)
   :gpidy (new-gpid (- kp) ti td ts)})

(defn set-pdf
  [{:keys [kp tf td ts]}]
  {:ku (+ (m/exp (- (/ ts tf))) (/ td tf))
   :gpdf (new-gpdf kp tf td ts)})

;placing ti instead of m/inf improved the disturbance rejection
;but it also introduced steady state error
(defn set-i-pd
  [{:keys [kp ti td ts]}]
  {:gpidy (new-gpid (- kp) m/inf td ts)
   :gi (* kp ts (/ ti))}) ;use new-gpid without pd @todo

(defn set-pid2
  [{:keys [b c kp ti td ts]}]
  (assert (not (zero? b)) "Use I_PD instead")
  {:gpidr (new-gpid (* kp b) (* ti b) (* td (/ c b)) ts)
   :gpidy (new-gpid (- kp) ti td ts)})

;problem: pidw sets! u-1 on act, so it won't be a pure function
;so every signal I add is the same situation...
;basically I want all functions that update signals, on step-next
;maybe some special keys to defctrl can specify the chain of calls
;this also means dropping the ICtrl protocol, for + generality
;the goal is kinkness at the level of common.processing @todo
(defn set-pidw
  [{:keys [kp ti td ts] :as ctrl}]
  (merge (set-pid ctrl) {:si (/ ts ti) :u-1 0.0}))

;initializing u-1 here is questionable. @bug
(defn set-pi-dw
  [{:keys [kp ti td ts] :as ctrl}]
  (merge (set-pi-d ctrl) {:si (/ ts ti) :u-1 0.0}))

(defn upid
  [{:keys [gpid e u]}]
  (reduce + (nth u -1) (map * (rseq e) gpid)))

(defn upi-d
  [{:keys [gpi gpidy r y u]}]
  (reduce + (nth u -1) 
    (interleave
      (map * (rseq r) gpi)
      (map * (rseq y) gpidy))))

(defn updf
  [{:keys [ku gpdf e u]}]
  (reduce + (* ku (nth u -1)) (map * (rseq e) gpdf)))

(defn ui-pd
  [{:keys [gi gpidy y e u]}]
  (reduce + (nth u -1)
    (cons (* gi (nth e -1))
          (map * (rseq y) gpidy))))

;very very very similar to upi-d. maybe they can be the same
;the trick is to name things by their relation with the signals
;so that similar blocks can share the same namings, and fns @todo
(defn upid2
  [{:keys [gpidr gpidy r y u]}]
  (reduce + (nth u -1)
    (interleave
      (map * (rseq r) gpidr)
      (map * (rseq y) gpidy))))

(defn rdefault [ctrl r] r)
(defn edefault [{:keys [r y]}] (- (nth r -1) (nth y -1)))
(defn udefault [ctrl] (nth (:u ctrl) -1))
 
;drop the use o protocols in favor of signals @todo
;then maybe there will be a single function step of protocol ICtrl
;which is basically the connections of a block diagram
;and the map of functions groups the blocks themselves @todo
;also, remove the need for ITune (nobody likes it anyway)
;the extra labor of having to specify extra props is cumbersome.
;as a side effect, not specifying enforces imutability @bug?

;@todo
;ok! so for every key I should create a prop with the same name
;the definition is a vector of props to their update function
;defctrl will also construct a correspondent step-next function
;so step-next is a protocol.
;in short, the keys will be the state, and the values their update
;each update will happen in sequence.
;a reset function will also be created!

;@todo
;then I could have meta-data for non-ring props
; (defctrl RelayRev
;   ^{:default {inf -0.01 sup 0.01 eps 0}
;     :initial (into (ring-buffer 3) [0 0 0])}
;    [:e err :y out :u act])
;y,e,r,u have special treatment (they have default updates)
;r in particular is the only fn that will expect two inputs
;defctrl should also create a new-pid fn
; (defctrl PID [:u u-pid])
; (new-pid {:gpid (gpid kp ti td ts)})

(defn- defctrl-fn
  ([t m] (defctrl-fn t [] m))
  ([t prop m] ;having ts separate from n is important. (?)
   (let [prop (into '[k n ts r y d e u uo enabled] prop)
         this (gensym "this")]
     `(defmutable ~t ~prop
        ITune
        (tune [~this]
          ~(if (contains? m :tune)
             (list (:tune m) this)))
        ICtrl
        (filt [~this r#]
          (~(get m :r `rdefault) ~this r#))
        (diff [~this]
          (~(get m :e `edefault) ~this)) 
        (act [~this]
          (~(get m :u `udefault) ~this))))))

(defmacro defctrl [t & args]
  (apply defctrl-fn t args))

(defctrl PIDCtrl {:tune set-pid :u upid})
(defctrl PI_DCtrl {:tune set-pi-d :u upi-d})
(defctrl PDFCtrl {:tune set-pdf :u updf})
(defctrl I_PDCtrl {:tune set-i-pd :u ui-pd})
(defctrl PID2Ctrl {:tune set-pid2 :u upid2})

(defn set-pi-pd
  [{:keys [kp ti td z ts]}]
  {:gy [(* kp td (/ ts)) (* kp td (m/exp (* z ts -1)) (/ ts) -1)]
   :gpi (new-gpid kp ti 0.0 ts)
   :u1-1 0.0}) ;@side-effect

;somehow PID2 already do this?
;and I only need one memory for u (and not the one for u1-1)
(defmutable PI_PDCtrl ;maybe this could be a PID2 @todo
  [n ;period in deltaTime units
   k ;counter in deltaTime units
   ts
   kp ti td z gy gpi
   u1-1 ;last C1 actuation
   #_RingBuffer r  ;reference
   #_RingBuffer y  ;output
   #_RingBuffer e  ;error
   #_RingBuffer u ;input (saturated)
   d ;disturbance
   enabled
   uo]
  ITune
  (tune [ctrl] (set-pi-pd ctrl))
  ICtrl
  (filt [ctrl r'] r')
  (diff [ctrl] (edefault ctrl))
  (act [ctrl]
    (let [u1 (reduce + u1-1 (map * (rseq e) gpi))
          u2 (reduce + (map * (rseq y) gy))]
      (set! (.u1-1 ctrl) u1) ;@side-effect
      (- u1 u2))))

;https://www.mathworks.com/help/slcontrol/ug/create-i-pd-and-pi-d-controllers.html
;I_PD is in a good position cause there is no need to arc r.
;PID2 it's a weighted mix between PI-D and I-PD

(defmutable PIDWCtrl
  [n ;period in deltaTime units
   k ;counter in deltaTime units
   ts
   d
   umax ;upper saturation
   umin ;lower saturation
   u-1 ;last input (unsaturated)
   si ;fator integral do erro por saturação
   gpid ;gain vector
   #_RingBuffer r  ;reference
   #_RingBuffer y  ;output
   #_RingBuffer e  ;error
   #_RingBuffer u ;input (saturated)
   enabled
   uo]
  ITune
  (tune [ctrl] 
    (set-pidw ctrl))
  ICtrl
  (filt [ctrl r'] r')
  (diff [ctrl] (edefault ctrl))
  (act [ctrl]
    (as-> (+ u-1 (* (- (nth u -1) u-1) si)) u-1
          (reduce + u-1 (map * (rseq e) gpid))
          (do (set! (.u-1 ctrl) u-1)
              (Math/Min (Math/Max u-1 umin) umax)))))

(defmutable PI_DWCtrl
  [n ;period in deltaTime units
   k ;counter in deltaTime units
   ts
   d
   umax ;upper saturation
   umin ;lower saturation
   u-1 ;last input (unsaturated)
   si ;fator integral do erro por saturação
   gpi ;gain vector
   gpidy ;gain vector
   #_RingBuffer r  ;reference
   #_RingBuffer y  ;output
   #_RingBuffer e  ;error
   #_RingBuffer u ;input (saturated)
   enabled
   uo]
  ITune
  (tune [ctrl]
    (set-pi-dw ctrl))
  ICtrl
  (filt [ctrl r'] r')
  (diff [ctrl] (edefault ctrl))
  (act [ctrl]
    (as-> (+ u-1 (* (- (nth u -1) u-1) si)) u-1
          (reduce + u-1
            (interleave
              (map * (rseq r) gpi)
              (map * (rseq y) gpidy)))
          (do (set! (.u-1 ctrl) u-1)
              (Math/Min (Math/Max u-1 umin) umax)))))

(defmutable COMP_DCtrl
  [n ;period in deltaTime units
   k ;counter in deltaTime units
   ts
   kp ke-1 ku kd
   u1-1 ;last C1 actuation
   #_RingBuffer r  ;reference
   #_RingBuffer y  ;output
   #_RingBuffer e  ;error
   #_RingBuffer u ;input (saturated)
   d ;disturbance
   enabled
   uo]
  ITune
  (tune [ctrl]) ;for now
  ICtrl
  (filt [ctrl r'] r')
  (diff [ctrl] (edefault ctrl))
  (act [ctrl]
    (let [u2 (* kd (- (nth y -1) (nth y -2)) (/ ts))
          u1 (+ (* kp (- (nth e -1) (* ke-1 (nth e -2)))) 
                (* ku u1-1))]
      (set! (.u1-1 ctrl) u1) ;@side-effect
      (- u1 u2))))

(defn new-comp-d [kp ke-1 ku kd n]
  (let [ts   (* n Time/fixedDeltaTime)
        ring (into (ring-buffer 3) [0.0 0.0 0.0])
        base {:ts ts :d 0.0 :u1-1 0.0 :n n :k 0 :enabled true :uo ring}
        ryue (zipmap [:r :y :u :e] (repeat 4 ring))
        conf (merge {:kp kp :ke-1 ke-1 :ku ku :kd kd} base ryue)]
    (map->COMP_DCtrl conf)))

;maybe I should return a basic ctrl map here, with the rings
;and then for every ctrl create a constructor (new-*) @todo
;that would eliminate the need for the ITune protocol.
;so I would drop both protocols in favor of a map of signals
;this map would be an array map from the signal key to it's fn
;maybe I can grab some inspiration from cycle.js @todo
(defn new-pid
  ([kp ti td n] (new-pid kp ti td n map->PIDCtrl))
  ([kp ti td n ctor & {:as opt}]
   (let [ts   (* n Time/fixedDeltaTime)
         ring (into (ring-buffer 3) [0.0 0.0 0.0])
         base {:ts ts :d 0.0 :n n :k 0 :enabled true :uo ring}
         ryue (zipmap [:r :y :u :e] (repeat 4 ring))
         conf (merge {:kp kp :ti ti :td td} base ryue opt)]
     (->> (ctor conf) tune (merge conf) ctor))))
; fix the need of having two ctor here @todo

(defn reset [ctrl]
  (let [buffers [(.r ctrl) (.e ctrl) (.e ctrl) (.u ctrl)]
        uo (mutable (snapshot (.uo ctrl)))]
    (dotimes [_ (apply max (map count buffers))]
      (step-next ctrl (first uo) (first uo)))
    (if (instance? PI_PDCtrl ctrl) ;bad sign @wip
      (set! (.u1-1 ctrl) 0.0))
    (set! (.u ctrl) uo)))

;; showcase of the goal of breaking it up in fns
;; + it seems easier to handle small definitions at the repl
;; the goal is to create new controllers in-game @todo

(defn rf1
  [{:keys [kf r]} in]
  (+ (* kf (nth r -1)) (* (- 1 kf) in)))

(defn ekr 
  [{:keys [kr r y]}]
  (- (* kr (nth r -1))
     (nth y -1)))

(defn uku
  [{:keys [ku gpid e u]}]
  (reduce +
    (* ku (nth u -1))
    (map * (rseq e) gpid)))

(defctrl CpCtrl {:tune set-pid :e ekr :u uku})
(defctrl LCpCtrl {:tune set-pid :r rf1 :e ekr :u uku})

;; RELAY

;this is more like a flip flop
;potentialy wrong initial value @bug
(defn relay-rev
  [{:keys [inf sup eps e r y u]}]
  (let [r-1 (nth r -1)
        y-1 (nth y -1)
        e-1 (nth e -1)]
    (cond
      (zero? r-1)
      (if (pos? y-1) sup inf) ;re-write this without u -1 @todo
      (and (neg? (* r-1 y-1)) (< (m/abs e-1) eps))
      (cond
        (= (nth u -1) inf) sup
        (= (nth u -1) sup) inf)
      :default
      (nth u -1))))

(defn relay-eps
  [{:keys [inf sup eps e u]}]
  (let [e-1 (nth e -1)]
    (cond
      (> e-1 eps) sup
      (< e-1 (- eps)) inf
      :else (nth u -1))))

(defctrl RelayCtrl [inf sup eps] {:u relay-eps})
(defctrl RelayRevCtrl [inf sup eps] {:u relay-rev})

(defn new-relay
  ([amp eps n] (new-relay amp (- amp) eps n map->RelayCtrl))
  ([sup inf eps n] (new-relay sup inf eps n map->RelayCtrl))
  ([sup inf eps n ctor]
   (let [ts   (* n Time/fixedDeltaTime)
         ring (conj (ring-buffer 1) 0)
         base {:ts ts :d 0.0 :n n :k 0 :eps eps :sup sup :inf inf :enabled true :uo ring}
         ryue (zipmap [:r :y :u :e] (repeat 4 ring))
         ctrl (ctor (merge base ryue))]
     (doto ctrl ;choose the initial act based on y @todo
       (as/sets! u (conj (.u ctrl) sup))))))

;; CTRL

;should be a zero order hold or something
(defn set-ctrl [^GameObject gob k] ;should be the very first
  (let [{:keys [props]} (state gob)]
    (doseq [[p off] (zipmap props (state gob k))]
      (state+ p :spin/set off))))

(defn add-ctrl [^GameObject gob k] ;must be called once @bug
  (let [{:keys [props] :ctrl/keys [euler height yaw pitch roll]}
        (state gob)
        [u0 u1 u2 u3] (state gob k)
        x (- u0 u2)
        y (+ u0 u2 (- u1) (- u3))
        z (- u1 u3)
        h (+ u0 u1 u2 u3)]
    (if euler (set! (.d euler) (v3 x y z)))
    (if height (set! (.d height) h))
    (if pitch (set! (.d pitch) x))
    (if roll (set! (.d roll) z))
    (if yaw (set! (.d yaw) y))
    (doseq [[p off] (zipmap props (state gob k))]
      (update-state p :spin/set + off))))

(defn add-ref [^GameObject gob k]
  (if-cmpt (state gob :ref) [tr Transform]
    (let [[x y z h] (state gob k)]
      (.Translate tr (v3- (v3 0 h 0)) Space/World)
      (.Rotate tr (v3- (v3 x y z))))))

(def pm3 (->Martins3 10.0 4.5 true))
(def fmax (.Thrust pm3 (u->w 1) 0.0)) ;@hard
(def tmax (* fmax 0.325)) ;@hard

(defn add-input [^GameObject gob k] ;must be called once! @bug
  (with-cmpt gob [rb Rigidbody]
    (let [{:ctrl/keys [euler height yaw pitch roll]} (state gob)
          [x y z h] (state gob k)] ;maybe these ifs are dumb
      ; (if euler (set! (.d euler) (v3 x y z)))
      ; (if height (set! (.d height) h))
      ; (if pitch (set! (.d pitch) x))
      ; (if roll (set! (.d roll) z))
      ; (if yaw (set! (.d yaw) y))
      (.AddForce rb (v3 0 (* h fmax) 0))
      (.AddRelativeTorque rb (v3* (v3 x y z) tmax)))))

;todo: remember to put this on TCC
;this solves only half the problem.
;internaly, Unity changes the eulerAngles rep. to avoid gimble lock
;so angles near (v3 270 90 270) will swap to (v3 270 320 40) instantly
;both representing (aa -90 1 0 0).
;basically, unity eulerAngles returns a pitch with an unsigned arc 90,
;and when the pitch arc 180 is greater than 91 it adds 180 to both yaw and roll
(defn arc [^double base degree]
  (let [sign (Math/Sign base)
        base (Math/Abs base)
        arco (mod degree 360)
        unit (int (quot arco base))
        pair (quot (inc unit) 2)]
    (cond-> (- arco (* 2 pair base))
            (odd? pair)
            (* (- sign)))))

(defn arc-s [base deg]
  (let [base (- base)]
    (-> (mod deg 360)
        (/ base)
        (as-> mul
          (if (odd? (int (quot (inc mul) 2)))
            (* mul (- (Math/Sign base)))
            mul))
        (* 90) ;wtf
        (/ m/deg:rad)
        (Math/Sin)
        (* base))))

;http://noelhughes.net/uploads/quat_2_euler_paper_ver3.pdf
;todo: convert to left hand rule (cross, qq*, qv*)
; (defn euler-zxy [^Quaternion q]
;   (let [v30 (v3 0.0 1.0 0.0)
;         v31 (v3 0.0 0.0 1.0)
;         v3q (qv* q v30)
;         e1 (* (Math/Atan2 (- (.x v3q)) (.y v3q)) m/deg:rad)
;         e2 (* (Math/Asin (.z v3q)) m/deg:rad)
;         q1 (aa e1 0.0 0.0 1.0)
;         q2 (aa e2 1.0 0.0 0.0)
;         v31q12 (qv* (qq* q1 q2) v31)
;         v31q (qv* q v31)
;         m (Vector3/Dot v3q (Vector3/Cross v31q12 v31q))
;         e3 (->> (Vector3/Dot v31q12 v31q) (Math/Acos)
;                 (* (Math/Sign m) m/deg:rad))]
;     (v3 e1 e2 e3)))

(defmacro v3map [f & v3s]
  (let [fs (gensym "f")
        vs (repeatedly (count v3s) #(vary-meta (gensym "v") assoc :tag Vector3))
        xs (map #(list '.x %) vs)
        ys (map #(list '.y %) vs)
        zs (map #(list '.z %) vs)]
    `(let [~fs ~f ~@(interleave vs v3s)]
       (v3 (~fs ~@xs) (~fs ~@ys) (~fs ~@zs)))))

;probleminha quando um angulo muda de -180 para 180
;https://robotics.stackexchange.com/questions/4836/how-to-control-pid-yaw
;nice answer given by Ian and Shahbaz. and the my asnwer:
;make it modular 90, but with the circular nature
;it adds two points of equilibrium (two zeros)
;the first one is unstable, the second is stable
;http://www.boldmethod.com/learn-to-fly/aerodynamics/3-types-of-static-and-dynamic-stability-in-aircraft/

;nothing works! new strategy: choose minimum arc, only change r
;any change in y, due to it crossing 180, -180 has to be reverted
;works 'fine' for yaw now, but what about mortal flips? @TODO
(defmutable Euler2Ctrl
  [n ;period in deltaTime units
   k ;counter in deltaTime units
   ts
   d
   #_ICtrl roll
   #_ICtrl yaw
   #_ICtrl pitch
   #_ICtrl relay-r
   #_ICtrl relay-y
   #_ICtrl relay-p
   #_RingBuffer r  ;reference
   #_RingBuffer y  ;output
   #_RingBuffer e  ;error
   #_RingBuffer u ;input
   enabled
   uo]
  ICtrl
  (filt [ctrl r'] r')
  (diff [ctrl]
    (v3map arc (v3 -180) (v3- (nth r -1) (nth y -1))))
  (act [ctrl]
    (as/let [r (nth r -1)
             y (nth y -1)
             r-1 (v3map arc (v3 90) r)
             y-1 (v3map arc (v3 -180) y)
             ysx (step-next relay-p (nth (.y relay-p) -1) (.x y-1))
             ysy (step-next relay-y (nth (.y relay-y) -1) (.y y-1))
             ysz (step-next relay-r (nth (.y relay-r) -1) (.z y-1))
             y-1x (arc 180 (.x y-1))
             y-1y (arc 180 (.y y-1))
             y-1z (arc 180 (.z y-1))
             usx (if (pos? (* ysx (.x y-1))) 1.0 -1.0)
             usy (if (pos? (* ysy (.y y-1))) 1.0 -1.0)
             usz (if (pos? (* ysz (.z y-1))) 1.0 -1.0)]
      (v3 (* usx (step-next pitch (.x r-1) (* ysx y-1x)))
          (* usy (step-next yaw (.y r-1) (* ysy y-1y)))
          (* usz (step-next roll (.z r-1) (* ysz y-1z)))))))

(defmutable EulerCtrl
  [n ;period in deltaTime units
   k ;counter in deltaTime units
   ts
   d
   #_ICtrl roll
   #_ICtrl yaw
   #_ICtrl pitch
   #_RingBuffer r  ;reference
   #_RingBuffer y  ;output
   #_RingBuffer e  ;error
   #_RingBuffer u ;input
   enabled
   uo]
  ICtrl
  (filt [ctrl r'] r')
  (diff [ctrl] ;works well if open loop has an integrator
    (v3map arc (v3 90 90 90) (v3- (nth r -1) (nth y -1))))
  (act [ctrl]
    (as/let [y-1 (v3- (nth e -1))]
      (v3 (step-next pitch 0.0 (.x y-1))
          (step-next yaw 0.0 (.y y-1))
          (step-next roll 0.0 (.z y-1))))))

;todo: clean this and the classes
(defn new-euler
  ([] (new-euler nil nil nil map->Euler2Ctrl))
  ([t] (new-euler nil nil nil t))
  ([x y z] (new-euler x y z map->Euler2Ctrl))
  ([x y z ctor & {n :n :as opt}]
   (let [n    (or n 1)
         ts   (* n Time/fixedDeltaTime)
         ring (conj (ring-buffer 1) (v3 0.0))
         base {:ts ts :d 0.0 :n n :k 0 :pitch x :yaw y :roll z 
               :enabled true :uo ring}
         rele {:relay-p (new-relay 1 -1 180 n map->RelayRevCtrl) 
               :relay-y (new-relay 1 -1 180 n map->RelayRevCtrl)
               :relay-r (new-relay 1 -1 180 n map->RelayRevCtrl)}
         ryue (zipmap [:r :y :u :e] (repeat 4 ring))]
     (ctor (merge base rele ryue opt)))))

;because we're using act-e here, the three unstable eq.
;become three unstable circunferences. in order to avoid them
;1. the interaction between axis has to be considered.
;for instance, restoring pitch (which is on yz plane)
;may influence the control of yaw or roll (question mark)
;I might have isolated the problem: pull the lever at 10° on
;yaw, then force the quad to 180° on pitch. you will see it
;turn counter-clockwise first and at ~ 90° it changes and
;turns clockwise until reaching ~ 0°. it might even change
;again at -90°.
;2. we have to look at what happens with the pid derivative
;when the input changes it's rate instantly, and see if the
;way of correction doesn't changes from clock to counter.
;attempt to solve 2: whenever there is an abrupt change of
;derivative, we will keep the derivative portion constant,
;and only change once it's derivative is mild again
;another attempt: change the reference signal so that it
;changes with continuous velocity, and it has zero velocity
;at 90° and -90°. I can't overdue though cause it might turn
;the control non linear. a nice canditate is the sin function
;spoiler: it wasn't enough to solve the switching correction
;the obvious way should be putting the derivative only on y
;let's see how that goes

; use FromToRotationInstead
; (defn qinv [^Vector3 u ^Vector3 v]
;   (let [w (+ 1 (Vector3/Dot u v))]
;     (if (< w 1e-6)
;       (qt (- (.z u)) (.x u) 0.0 0.0)
;       (let [uxv (Vector3/Cross u v)
;             mag (Vector4/Magnitude (v4 w (.x uxv) (.y uxv) (.z uxv)))]
;         (qt (/ (.x uxv) mag) (/ (.y uxv) mag) (/ (.z uxv) mag) (/ w mag))))))

;there is a coupling between pitch and roll, whenever there is yaw
;maybe the cross relationship between ux and uz is related
;however, unity does not simulate
;IMPORTANT: @tcc
; even if the ctrls perform well with SISO they can behave poorly
; when in MIMO (try a step on yaw and roll at the same time)
(defn euler-ctrl [^GameObject gob k]
  ;put first let in a switch-ctrl hook @todo
  (let [ce (state gob :ctrl/euler)] 
    (set! (.yaw ce) (state gob :ctrl/yaw))
    (set! (.roll ce) (state gob :ctrl/roll))
    (set! (.pitch ce) (state gob :ctrl/pitch)))
  (let [{:keys [ref props]} (state gob)
        y (.. gob transform rotation eulerAngles)
        r (.eulerAngles (Quaternion/Inverse (.. ref transform localRotation)))
        u (step-next (state gob k) r y)
        ux (.x u) uy (.y u) uz (.z u)
        p0 (+ (/ ux +2) (/ uy +4))
        p1 (+ (/ uz +2) (/ uy -4))
        p2 (+ (/ ux -2) (/ uy +4))
        p3 (+ (/ uz -2) (/ uy -4))]
    (doseq [[p pk] (zipmap props [p0 p1 p2 p3])]
      (update-state p :spin/set + pk))))

(defn height-ctrl [^GameObject gob k]
  (as/let [{:keys [ref props]} (state gob)
           y (.. gob transform position y)
           r (.. ref transform localPosition y)
           u (step-next (state gob k) (- r) y)]
    (doseq [p props]
      (update-state p :spin/set + (/ u 4)))))

(def max-tilt 5.0)
(def min-dist 0.5)

;for some reason, which it doesn't feel too strange,
;the controller becomes unstable for |pitch| or |roll| > 90
;spoiler: is because unity hijacks the eulers
;update: it is solved in Euler2Ctrl with the help of arc
(defn panic? [^GameObject gob]
  (as/let [(as/with-cmpt tr Transform) gob
           (as/o :props [position rotation]) tr
           euler (.eulerAngles rotation)]
    (or (> (arc 180 (.x euler)) (inc max-tilt))
        (> (arc 180 (.z euler)) (inc max-tilt)))))

;cool thing to put on tcc:
;omni (with relay) can have limit cycles circling around
;goto (with PID) can have limit cycles doing a infinite symbol
;how to amortize both?
;an obvious way is to implement a break-like drag (break-ctrl)
(defn plane-ctrl [^GameObject gob k]
  (as/let [{:keys [ref]} (state gob)
           (as/with-cmpt qtr Transform rb Rigidbody) gob
           (as/with-cmpt rtr Transform) ref
           (as/o :props [x z]) (. qtr position)
           xr (.. rtr localPosition x)
           zr (.. rtr localPosition z)
           err (v3- (v3 xr 0.0 zr) (v3 x 0.0 z))
           fow (. qtr forward)
           vdir (Vector3/SignedAngle fow err up)
           yawe (arc 45 vdir)
           yerr (qq* (.rotation qtr) (angle-axis (- yawe) up))
           derr (.magnitude err)
           vn (step-next (state gob k) min-dist derr)
           vd (angle-axis vn (q* (angle-axis 90 up) err))]
    ; todo: think about composing rotations
    (if-not (panic? gob)
      (set! (.localRotation rtr)
        (case k
          :ctrl/look yerr ;@broken: too slow, not exact
          :ctrl/omni vd ;good!
          :ctrl/goto (qq* vd yerr)))))) ;@broken: flips out on yaw, roll

;the goal is to zero the speed, but it can't ever succeed 100%
;so let's use a PD controller for this one
;it seems to only mitigate the too close circling problem.
;but it works well for straight lines
;todo: think how to stop a the circle 'inertia'
(defn break-ctrl [^GameObject gob k]
  (as/let [{:keys [ref]} (state gob)
           (as/with-cmpt rb Rigidbody) gob
           (as/with-cmpt rtr Transform) ref
           vel (v3 (.. rb velocity x) 0.0 (.. rb velocity z))
           derr (min (.magnitude vel) max-tilt)
           vn (step-next (state gob k) 0.0 derr)
           vb (angle-axis vn (q* (angle-axis -90 up) vel))]
    (set! (.localRotation rtr)
      (qq* vb (.localRotation rtr)))))

;; DEV

(load "quad/scene")
(load "quad/editor")