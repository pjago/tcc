(do (in-ns 'pjago.quad)
    (require '[common.repl :as re]
             '[common.processing :as x :refer [render]])
    (import '[UnityEngine.SceneManagement SceneManager]
            '[XaertV Drone]))

;; HOOKS+ (order of execution is order they are added)
;this should be at edn
(as/let [(re/source ::quad ::arm ::propeller) GameObject
         props (re/sources (find-ns 'pjago.quad.prop))]
  (hook+ quad :fixed-update :ctrl #'spin-ctrl)
  (state+ quad :ctrl (repeatedly 3 #(new-pid 1.0 0.5 0.0 10)))
  (doseq [prop (cons propeller props)]
    (hook+ prop :on-transform-parent-changed :snap/connect #'snap-joint)
    (hook+ prop :on-joint-break :snap/break #'snap-joint)
    (hook+ prop :fixed-update :spin #'kinematic)
    (hook+ prop :fixed-update :speed #'kinematic)
    (hook+ prop :fixed-update :thrust #'dynamic)
    (hook+ prop :fixed-update :drag #'dynamic)
    (state+ prop :aero (->Stables 10.0 4.5 true))
    (state+ prop :drag 0.0)
    (state+ prop :thrust 0.0)
    (state+ prop :spin 0.0)
    (state+ prop :speed 0.0)))

;; IMMEDIATE MODE

(defn new-propeller []
  (as/let [(re/clone! ::propeller) GameObject]
    (re/render! [propeller])))

(defn new-motor []
  (as/let [(re/clone! ::motor) GameObject]
    (re/render!
      [(new-propeller)]
      [motor])))

(defn new-arm [material]
  (as/let [(re/source material) Material
           (re/clone! ::arm) GameObject
           (re/clone! ::hand ::forearm ::elbow) GameObject]
    (doseq [gob [hand forearm elbow]]
      (with-cmpt gob [mr MeshRenderer]
        (set! (.-sharedMaterial mr) material)))
    (re/render!
      [arm
       [(new-motor)
        hand
        forearm
        elbow]])))

(defn new-quad [materials]
  (as/let [(re/clone! ::quad ::base) GameObject
           (as/with-cmpt qrb Rigidbody) quad
           (as/with-cmpt bfj FixedJoint) base
           arms (mapv new-arm (take 4 (cycle materials)))
           props (filter #(re/o= ::propeller %) (mapcat children arms))]
    (set! (.-connectedBody bfj) qrb)
    (mirror! arms (v3 0) up)
    (doseq [arm arms]
      (re/bind-disabled arm
        (as/let [(as/with-cmpt afj FixedJoint) arm]
          (set! (.-connectedBody afj) qrb))))
    (state+ quad :props (vec props))
    (mapv #(set! (.-clockwise (state %1 :aero)) %2) 
          props 
          (cycle [true false]))
    (re/render!
      [quad
       [arms
        ::ushell
        ::lshell
        base]])))

(do "background"
  (re/clear-all!)
  (re/render!
    [:light-main]
    [:camera-main]
    [:camera-top]
    [:ground]))

(do "lock sans" 
  (re/clear!)
  (as/let [(re/search :camera-main :camera-top) GameObject
           (re/clone! :sansbox) GameObject
           (as/with-cmpt top-cam Camera) camera-top
           (as/with-cmpt sdr Drone) sansbox]
    (mario-cam! camera-main sansbox)
    (set! (.-projectWith sdr) top-cam)
    (re/render!
      [camera-top]
      [camera-main]
      [sansbox])))

(do "simple prop"
  (re/clear!)
  (as/let [(re/search :camera-main) GameObject
           (re/clone! ::quad) GameObject
           (re/source :sansbox ::qp/fixed) GameObject
           propeller (re/clone! (new-propeller) fixed)]
    (re/clone! quad sansbox)
    (mario-cam! camera-main quad)
    (state+ quad :props [propeller])
    (state+ quad :goal [929])
    (re/render!
      [camera-main]
      [quad
       [propeller]])))

(do "prop with arm"
  (re/clear!)
  (as/let [(re/search :camera-main) GameObject
           (re/clone! ::quad) GameObject
           (re/source :sansbox ::qp/fixed) GameObject
           arm (new-arm :plastic-white)
           (as/with-cmpt rb Rigidbody) quad
           (as/with-cmpt fj FixedJoint) arm
           propeller (re/leaf arm ::propeller)]
    (re/bind-disabled arm
      (set! (.-connectedBody fj) rb))
    (re/bind-disabled propeller
      (re/clone! propeller fixed)
      (set! (.. propeller -transform -localPosition)
            (v3 0 0.38 8.5447)))
    (re/bind-disabled quad
      (re/clone! quad sansbox)
      (spin-0 quad 30 (v3 1 0 0)))
    (mario-cam! camera-main quad)
    (state+ quad :props [propeller])
    (state+ propeller :spin 1200) ;957
    (re/render!
      [camera-main]
      [quad
       [arm
        [propeller]]])))

(do "quadcopter"
  (re/clear!)
  (as/let [(re/search :camera-main) GameObject
           quad (new-quad [:plastic-white :plastic-red])]
    (mario-cam! camera-main quad)
    (state+ quad :goal (repeat 4 550))
    (if-cmpt quad [gui (hook-types :on-gui)]
      (set! (.-enabled gui) true))
    (re/destroy! (re/leaf quad ::base))
    (re/render!
      [quad]
      [camera-main])))