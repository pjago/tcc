(ns pjago.quad
  (:use arcadia.core arcadia.linear pjago.hooks)
  (:import [UnityEngine Application GameObject Transform Vector3 Matrix4x4 Quaternion Space ForceMode Time Rigidbody JointMotor Joint HingeJoint FixedJoint Material MeshRenderer Camera BoxCollider SphereCollider RigidbodyConstraints Component Rect GUI]
           ArcadiaState ArcadiaBehaviour)
           ; [amalloy.ring_buffer RingBuffer])
  (:require [common.math :as m]
            ; [magic.api :as w]
            [arcadia.sugar :as as]
            ; [amalloy.ring-buffer :refer [ring-buffer]]
            [clojure.string :as str]
            [common.processing :as x]
            [pjago.watch :as watch :refer [ring-buffer]]
            [pjago.quad.prop :as qp]))

; (import '[amalloy.ring_buffer RingBuffer]) ;wut

;; CONST

(def up (v3 0.0 1 0.0)) ;propeller is facing up
(def right (v3 1 0.0 0.0))
(def forward (v3 0.0 0.0 1))
(def rho 1.2754) ;[kg:m³] CNPT air density

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
            W (m/* (.inverse I) i (.angularVelocity rb))
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
(defn missing-physx [^GameObject gob k]
  (with-cmpt gob [tr Transform rb Rigidbody]
    (case k
      :unstable-rotation
      (let [i (Matrix4x4/Scale (.inertiaTensor rb))
            w (.angularVelocity rb)
            M (trs (v3 0) (.rotation tr) (v3 1))
            W (m/* (.inverse M) (.angularVelocity rb))
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
(definterface IAero
  (^double Thrust [^double rpm ^double forward_speed])
  (^double Drag [^double rpm ^double forward_speed])
  (^double Cp [^double advance_ratio]) ;drag coefficient
  (^double Ct [^double advance_ratio]) ;thrust coefficient
  (^double Ve [^double rpm])) ;exit velocity

;broken: uses quad speed, not prop speed
(defn kinematic [^GameObject gob k]
  (if-cmpt (.. gob transform root) [rb Rigidbody]
    (as/let [(as/o :props [velocity]) rb
             M (.. gob -transform -worldToLocalMatrix)]
      (state+ gob k
        (case k
          :speed
          (m/dot (.MultiplyVector M velocity) up)
          :spin
          (or (state gob :spin/set) 0))))
    (state+ gob k 0)))

;broken: adds force, should leave that to quad-physics
(defn dynamic [^GameObject gob k]
  (if-cmpt (.. gob transform root) [rb Rigidbody]
    (as/let [(as/o :state [aero spin speed]) gob]
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
          (state+ p :speed (m/dot v up))
          (state+ p :spin (state p :spin/set))))))) ;immediate spin

(defn dynamico [^GameObject gob k]
  (as/let [(as/with-cmpt rb Rigidbody) gob]
    (doseq [p (state gob :props)]
      (as/let [(as/o :state [aero spin speed]) p
               (as/with-cmpt tr Transform) p
               T (.Thrust aero spin speed)
               P (.Drag aero spin speed)]
        (state+ p :thrust T)
        (state+ p :drag P)))))

(defn quad-physx [^GameObject gob k]
  (as/let [(as/with-cmpt rb Rigidbody) gob
           acc-drag (volatile! (v3 0))]
    (doseq [p (state gob :props)]
      (as/let [(as/o :state [thrust drag]) p
               (as/with-cmpt tr Transform) p
               pos (.-position tr)
               T (v3* (. tr -up) thrust)
               P (v3* (. tr -up) drag)]
        (.AddForceAtPosition rb T pos)
        (vswap! acc-drag v3+ P)))
    (.AddTorque rb @acc-drag)))

; todo: http://m-selig.ae.illinois.edu/props/propDB.html#APC
; https://physics.stackexchange.com/questions/31811/calculate-quadrotor-propeller-torque-due-to-aerodynamic-drag
(defmutable UIUC 
  [^float diameter ^float pitch ^boolean clockwise]
  IAero ;todo: use diameter and pitch
  (Ve [dyn w] ;todo: exit profile, # of blades, shape
    (* w pitch m/m:rad.in))
  (Ct [_ j] ;[10x5] aprox.
    (max 0 (m/lmap j 0.085 0.65 0.095 0)))
  (Cp [_ j] ;[10x5] aprox.
    (if (> j 0.3)
      (max 0 (m/lmap j 0.3 0.7 0.04 0.0075))
      0.04))
  (Drag [dyn w v]
    (let [va (- (.Ve dyn w) v) ;true air speed
          j (/ va (* w diameter m/m:in))] ;advance ratio
      (* 4.16231426E-7 rho (m/abs va) w (/ m/tau)
         (m/pow diameter 5)
         (.Cp dyn j)
         (if clockwise 1 -1))))
  (Thrust [dyn w v]
    (let [va (- (.Ve dyn w) v) ;true air speed
          j (/ va (* w diameter m/m:in))] ;advance ratio
      (* 4.16231426E-7 rho va (m/abs w)
         (m/pow diameter 4)
         (.Ct dyn j)))))

; http://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html
(defmutable Stables 
  [^float diameter ^float pitch ^boolean clockwise]
  IAero 
  (Ve [dyn w] ;todo: exit profile, # of blades, shape
    (* w pitch m/m:rad.in))
  (Ct [_ j]
    (/ 1.00771633 (m/sqrt (* diameter pitch)) rho))
  (Cp [_ j] ;only a guess, not from Stables
    (/ 1.00771633 (m/sqrt (* diameter pitch)) rho 2))
  (Drag [dyn w v]
    (let [va (- (.Ve dyn w) v) ;true air speed
          j (/ va (* w diameter m/m:in))] ;advance ratio
      (* 4.19443207E-07 (m/abs va) w (/ m/tau) 0.5
         (m/pow diameter 4.5)
         (/ (m/sqrt pitch))
         (if clockwise 1.0 -1.0))))
  (Thrust [dyn w v]
    (let [va (- (.Ve dyn w) v)] ;true air speed
      (* 4.19443207E-07 va (m/abs w)
         (m/pow diameter 3.5) 
         (/ (m/sqrt pitch))))))

;; PID

(defn new-gpid [^double kp ^double ti ^double td ^double ts]
  (let [i (/ ts ti 2)
        d (/ td ts)]
    (mapv #(* % kp) 
          [(+ i d 1) 
           (- i (* d 2) 1) 
           d])))
      
(definterface IPID
  (SetPID [^double kp ^double ti ^double td]))

;todo: implement PID, Fuzzy, ANN, etc
(definterface ICtrl
  (Diff [])
  (Act []))

(defn step-next [ctrl r y]
  (if-not (zero? (.k ctrl))
    (do (set! (.k ctrl) (dec (.k ctrl)))
        (nth (.u ctrl) -1)) ;zero order hold
    (-> (as/sets! ctrl
          k (dec (.n ctrl))
          y (conj (.y ctrl) y)
          r (conj (.r ctrl) r)
          e (conj (.e ctrl) (.Diff ctrl))
          u (conj (.u ctrl) (.Act ctrl)))
        (nth -1))))

(defmutable PIDCtrl
  [n ;period in deltaTime units
   k ;counter in deltaTime units
   gpid ;gain vector
   #_RingBuffer r  ;reference
   #_RingBuffer y  ;output
   #_RingBuffer e  ;error
   #_RingBuffer u] ;input
  IPID
  (SetPID [ctrl kp ti td]
    (let [ts (* n Time/fixedDeltaTime)]
      (as/sets! ctrl gpid (new-gpid kp ti td ts))))
  ICtrl
  (Diff [ctrl]
    (- (nth r -1) (nth y -1)))
  (Act [ctrl]
    (reduce + (nth u -1) (map * (rseq e) gpid))))

(defmutable PI_DCtrl
  [n ;period in deltaTime units
   k ;counter in deltaTime units
   gpi ;gain vector
   gpidy ;gain vector
   #_RingBuffer r  ;reference
   #_RingBuffer y  ;output
   #_RingBuffer e  ;error
   #_RingBuffer u] ;input
  IPID
  (SetPID [ctrl kp ti td]
    (let [ts (* n Time/fixedDeltaTime)]
      (as/sets! ctrl
        gpi (new-gpid kp ti 0.0 ts)
        gpidy (new-gpid (- kp) ti td ts))))
  ICtrl
  (Diff [ctrl]
    (- (nth r -1) (nth y -1)))
  (Act [ctrl]
    (reduce + (nth u -1) 
      (interleave
        (map * (rseq r) gpi)
        (map * (rseq y) gpidy)))))

;I_PD is in a good position cause there is no need to arc r.
;but that in other hand makes the control rather slow
(defmutable I_PDCtrl
  [n ;period in deltaTime units
   k ;counter in deltaTime units
   gi ;integrator gain
   gpidy ;gain vector
   #_RingBuffer r  ;reference
   #_RingBuffer y  ;output
   #_RingBuffer e  ;error
   #_RingBuffer u] ;input
  IPID
  (SetPID [ctrl kp ti td]
    (let [ts (* n Time/fixedDeltaTime)]
      (as/sets! ctrl
        gi (* kp (/ ti) ts)
        gpidy (new-gpid (- kp) ti td ts))))
  ICtrl
  (Diff [ctrl]
    (- (nth r -1) (nth y -1)))
  (Act [ctrl]
    (reduce + (nth u -1)
      (cons (* gi (nth e -1))
            (map * (rseq y) gpidy)))))

;https://www.mathworks.com/help/slcontrol/ug/create-i-pd-and-pi-d-controllers.html
;that's a mix between PI-D and I-PD, with b and c as weights
(defmutable PID2Ctrl
  [n ;period in deltaTime units
   k ;counter in deltaTime units
   b ;weight for proportional reference
   c ;weight for derivative reference
   gpidr ;gain vector
   gpidy ;gain vector
   #_RingBuffer r  ;reference
   #_RingBuffer y  ;output
   #_RingBuffer e  ;error
   #_RingBuffer u] ;input
  IPID
  (SetPID [ctrl kp ti td]
    (let [ts (* n Time/fixedDeltaTime)]
      (assert (not (zero? b)) "Use I_PD instead")
      (as/sets! ctrl
        gpidr (new-gpid (* kp b) (* ti b) (* td (/ c b)) ts)
        gpidy (new-gpid (- kp) ti td ts))))
  ICtrl
  (Diff [ctrl]
    (- (nth r -1) (nth y -1)))
  (Act [ctrl]
    (reduce + (nth u -1)
      (interleave
        (map * (rseq r) gpidr)
        (map * (rseq y) gpidy)))))

(defn new-pid
  ([kp ti td n] (new-pid kp ti td n map->PIDCtrl))
  ([kp ti td n ctor & {:as opt}]
   (let [ring (into (ring-buffer 3) [0.0 0.0 0.0])
         ryue (zipmap [:r :y :u :e] (repeat 4 ring))]
     (doto (ctor (merge {:n n :k 0} ryue opt))
       (.SetPID kp ti td)))))

;; CTRL

;should be a zero order hold or something
(defn spin-ctrl [^GameObject gob k]
  (let [{:keys [props]} (state gob)]
    (doseq [[p off] (zipmap props (state gob :offset))]
      (state+ p :spin/set off))))

;todo: remember to put this on TCC
;this solves only half the problem.
;internaly, Unity changes the eulerAngles rep. to avoid gimble lock
;so angles near (v3 270 90 270) will swap to (v3 270 320 40) instantly
;both representing (aa -90 1 0 0).
;basically, unity eulerAngles returns a pitch with an unsigned arc 90,
;and when the pitch arc 180 is greater than 91 it adds 180 to both yaw and roll
(defn arc [^double base deg]
  (let [sign (Math/Sign base)
        base (Math/Abs base)
        deg (mod deg 360)
        mul (int (quot deg base))
        s (quot (inc mul) 2)]
    (cond-> (- deg (* s base 2))
            (odd? s)
            (* (- sign)))))

(defn arc-s [base deg]
  (let [base (- base)]
    (-> (mod deg 360)
        (/ base)
        (as-> mul
          (if (odd? (int (quot (inc mul) 2)))
            (* mul (- (Math/Sign base)))
            mul))
        (* 90)
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
  (let [vs (repeatedly (count v3s) #(vary-meta (gensym "v") assoc :tag Vector3))
        xs (map #(list '.x %) vs)
        ys (map #(list '.y %) vs)
        zs (map #(list '.z %) vs)]
    `(let [~@(interleave vs v3s)]
       (v3 (~f ~@xs) (~f ~@ys) (~f ~@zs)))))

;probleminha quando um angulo muda de -180 para 180
;https://robotics.stackexchange.com/questions/4836/how-to-control-pid-yaw
;nice answer given by Ian and Shahbaz. and the my asnwer:
;make it modular 90, but with the circular nature
;it adds two points of equilibrium (two zeros)
;the first one is unstable, the second is stable
;http://www.boldmethod.com/learn-to-fly/aerodynamics/3-types-of-static-and-dynamic-stability-in-aircraft/
(defmutable Euler2Ctrl
  [n ;period in deltaTime units
   k ;counter in deltaTime units
   ^ICtrl roll
   ^ICtrl yaw
   ^ICtrl pitch
   #_RingBuffer r  ;reference
   #_RingBuffer y  ;output
   #_RingBuffer e  ;error
   #_RingBuffer u] ;input
  ICtrl
  (Diff [ctrl]
    (v3map arc
      (v3 -180 90 -180)
      (v3- (nth r -1) (nth y -1))))
  (Act [ctrl]
    (as/let [base (v3 -180 90 -180)
             r-1 (v3map arc base (nth r -1))
             y-1 (v3- r-1 (nth e -1))]
      (v3 (step-next pitch (.x r-1) (.x y-1))
          (step-next yaw (.y r-1) (.y y-1))
          (step-next roll (.z r-1) (.z y-1))))))

(defmutable EulerCtrl
  [n ;period in deltaTime units
   k ;counter in deltaTime units
   ^ICtrl roll
   ^ICtrl yaw
   ^ICtrl pitch
   #_RingBuffer r  ;reference
   #_RingBuffer y  ;output
   #_RingBuffer e  ;error
   #_RingBuffer u] ;input
  ICtrl
  (Diff [ctrl]
    (v3map arc
      (v3 -180 90 -180)
      (v3- (nth r -1) (nth y -1))))
  (Act [ctrl]
    (as/let [y-1 (v3- (nth e -1))]
      (v3 (step-next pitch 0.0 (.x y-1))
          (step-next yaw 0.0 (.y y-1))
          (step-next roll 0.0 (.z y-1))))))

(defn new-euler
  ([x y z] (new-euler x y z map->EulerCtrl))
  ([x y z ctor & {:as opt}]
   (let [ring (conj (ring-buffer 1) (v3 0.0))
         ryue (zipmap [:r :y :u :e] (repeat 4 ring))]
     (ctor (merge {:n 1 :k 0 :pitch x :yaw y :roll z} ryue opt)))))

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
(defn euler-ctrl [^GameObject gob k]
  (as/let [(as/o :state [ref props]) gob
           y (.. gob transform rotation eulerAngles)
           r (.. ref transform localRotation eulerAngles)
           u (step-next (state gob k) (v3- r) y)
           ux (.x u) uy (.y u) uz (.z u)
           p0 (+ ux uz uy)
           p1 (+ (- ux) uz (- uy))
           p2 (+ (- ux) (- uz) uy)
           p3 (+ ux (- uz) (- uy))]
    (doseq [[p pk] (zipmap props [p0 p1 p2 p3])]
      (update-state p :spin/set + pk))))

(defn height-ctrl [^GameObject gob k]
  (as/let [(as/o :state [ref props]) gob
           y (.. gob transform position y)
           r (.. ref transform localPosition y)
           u (step-next (state gob k) (- r) y)]
    (doseq [p props]
      (update-state p :spin/set + u))))

;for some reason, which it doesn't feel too strange,
;the controller becomes unstable for |pitch| or |roll| > 90
(defn stable? [^GameObject gob]
  (as/let [(as/with-cmpt tr Transform) gob
           (as/o :props [position rotation]) tr
           euler (.eulerAngles rotation)]
    (not (or (> (.magnitude position) 1000)
             (> (arc 180 (.x euler)) 90)
             (> (arc 180 (.z euler)) 90)))))

(defn plane-ctrl [^GameObject gob k]
  (as/let [(as/o :state [ref props]) gob
           (as/with-cmpt qtr Transform rb Rigidbody) gob
           (as/with-cmpt rtr Transform) ref
           (as/o :props [x z]) (. rtr position)
           dist (.magnitude (v2 x z))
           pos (v3 x 0.0 z)
           fow (. qtr forward)
           vdir (Vector3/SignedAngle fow (v3- pos) up)
           yawg (.. gob transform rotation eulerAngles y)
           yawe (arc 45 vdir)
           yerr (angle-axis (- yawg yawe) up)
           max-angle 5.0 ;hard: max-angle, and dist
           vn (step-next (state gob k) 0 (min (- dist 0.5) max-angle))
           vd (angle-axis vn (q* (angle-axis -90 up) pos))
           vq (angle-axis (- yawg yawe) pos)]
    ; should be using transform rotate instead??   
    (set! (.localRotation rtr)
      (case k
        :plane/look yerr
        :plane/goto (qq* vq yerr vd)
        :plane/omni vd))))

;; DEV

(load "quad/editor")
(load "quad/scene")