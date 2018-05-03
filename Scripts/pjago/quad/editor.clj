(do (in-ns 'pjago.quad)
    (import '[UnityEngine GameObject Screen GUI Color Rect]))
  
;; HOOKS

(def new-rect (memoize #(Rect. %1 %2 %3 %4)))

(defn label [[x y w h] txt]
  (GUI/Label (new-rect x y w h) txt))

(defn aero-info [^GameObject gob k]
  (as/let [(as/o :state [thrust drag spin speed]) gob
           H (/ Screen/height 4)
           W (/ Screen/width 3)]
    (label [0 0 W H] (format "T: %.2f" (float thrust)))
    (label [0 15 W H] (format "P: %.2f" (float drag)))
    (label [0 30 W H] (format "w: %.2f" (float spin)))
    (label [0 45 W H] (format "v: %.2f" (float speed)))))

(defn ctrl-info [^GameObject gob k]
  (as/let [(as/o :state [props]) gob
           H (/ Screen/height 4)
           W (/ Screen/width 2)
           props (seq props)]
    (dotimes [i (count props)]
      (as/let [p (nth props i) (as/o :state [thrust drag]) p]
        (label [W (* i 15) W H]
          (format "T(%d): %.2f | P(%d): %.2f"
                  i (float thrust)
                  i (float drag)))))))

(defn euler-info [^GameObject gob k]
  (as/let [H (/ Screen/height 4)
           W (/ Screen/width 3)]
    (as/let [e (.. gob -transform -rotation -eulerAngles)]
      (label [W 0 W H] (format "roll: %.2f" (.-x e)))
      (label [W 15 W H] (format "yaw: %.2f" (.-y e)))
      (label [W 30 W H] (format "pitch: %.2f" (.-z e))))))