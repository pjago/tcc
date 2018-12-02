(in-ns 'pjago.quad)
(use 'clojure.pprint)

(def d 10)
(def p 4.5)
(def n 8500)
(def w (/ n m/rpm.s:rad))
(def v 0)
(def ve (* w p m/m:rad.in))
(def va (- ve v))
(def j (/ va (* w d m/m:in)))
(def udyn (->UIUC d p true))
(def sdyn (->Stables d p true))

[(.Ct udyn j)
 (.Ct sdyn j)]

[(.Cp udyn j)
 (.Cp sdyn j)]

(m// [(.Thrust udyn w v)
      (.Thrust sdyn w v)]
     m/g
     0.001)

[(.Drag udyn w v)
 (.Drag sdyn w v)]

(->> (new-pid 1 m/inf 0 5) 
     (iterate #(doto % (step-next 0 10)))
     (take 15)
     (mapv #(seq (.u %)))
     (pprint))

(comment [k :fixed-update] ;tests whether execution order is respected
  (->> (hook-types k)
       (s> ::quad)    ;'problem' here
       (.ifnInfos)    ;should probably think about testing
       (mapv #(.key %))
       (#(= (filterv (set %) (k quad-ex)) %))))

;I like test benches like these.
;but they unevitably get deleted everytime I clean the code
;I should put a rule where there is no code deletion
;all the tests should be moved to another file on the same ns
;and they should clojure.test or something
;@pjago, 04/20/2018
(comment [cube (create-primitive :cube)]
  ;should probably steal focus of the camera??
  ; (run! destroy (objects-named "Cube"))
  (with-cmpt cube [tr Transform rb Rigidbody]
    (hook+ cube :fixed-update :unstable-rotation #'missing-physx)
    (as/sets! tr
      localScale (v3 70 140 7)
      position (v3 0))
    (as/sets! rb
      useGravity false
      angularDrag 0.0)
    (.AddForceAtPosition rb
     (v3 0 0 0.5)
     (v3 0 70 5))))

(comment [T (GameObject. "T")]
         l (create-primitive :cube)
         _ (create-primitive :cube)
         (as/with-cmpt trb Rigidbody) T
         (as/with-cmpt _tr Transform) _
         (as/with-cmpt ltr Transform) l
  (child+ T l)
  (child+ T _)
  (as/sets! _tr
    localScale (v3 1 10 1)
    localPosition (v3 -0.5 0 0))
  (as/sets! ltr
    localScale (v3 4 1 1)
    localPosition (v3 2 0 0))
  (as/sets! trb
      useGravity false
      angularDrag 0.0))

(comment [b (GameObject. "my-balls")]
         b0 (create-primitive :sphere)
         b1 (create-primitive :sphere)
         (as/with-cmpt rb Rigidbody) b
         (as/with-cmpt tr0 Transform) b0
         (as/with-cmpt tr1 Transform) b1
  (hook+ b :fixed-update :unstable-rotation #'missing-physx)
  (set! (.position tr0) (v3 0))
  (set! (.position tr1) (v3 1 1 0))
  (child+ b b0)
  (child+ b b1)
  (as/sets! rb
    useGravity false
    angularDrag 0.0
    angularVelocity (v3 0.0 0.0 4.0)))
