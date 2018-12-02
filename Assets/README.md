This project is an integral part of my TCC (undergraduate thesis).
It simulates a real quadrotor drone, with the help of Unity, Arcadia, and Clojure.

You can use it by executing the latest build, and interacting with the toggles. The bottom slider changes the camera zoom, or changes the selected parameter while paused.

You can also write and eval your own scripts starting from the Clojure namespace `pjago.quad`. For REPL, you can use the Socket plugin in Sublime-Text 3, or just use the internal REPL.

**UI:**
' (1) : opens the internal REPL in the current namespace (issue #1)
&larr,&rarr,&darr,&uarr : disturbance OR reference for yaw and height
4,6,2,8 : disturbance OR reference for pitch and roll
CTRL : switch input from disturbance to reference (the cylinder below the quad)
SHIFT + TAB : switch player OR switch parameter back while paused
TAB : switch camera OR switch parameter forward while paused
ESC : closes the internal repl OR pauses and display save, quit, and restart
SAVE : will save the latest simulation state in the .\data folder
QUIT : quits the application without freezing (issue #2)
RESTART : attempts to reset the current player state

**REPL** (code defined in the `pjago.quad` namespace):
(in-ns 'pjago.quad) ;=> switch to the namespace where most fns are defined
(get-player) ;=> returns the current player, focused by the camera
(.. (get-player) transform position) ;=> UnityEngine.Vector3
(.. (get-player) transform rotation) ;=> UnityEngine.Quaternion
(render player) ;=> renders a new quad with the current settings
(relay-3) ;=> instantiates the euler-angle feedback relays (TCC WIP)

QUESTIONS?
Send an email to pedro.m4rtins@gmail.com