# AirplanePitchStabilizer
This Arduino sketch is a servo controller for a tail-less airplane that is stabilized about the pitch-axis with an angle-of-attack sensor. If tuned correctly, the result is an aircraft with a greater lift-to-drag ratio than what is otherwise possible with passive stability.  

To provide this stability enhancement, the code uses the signal an analog A.O.A. sensor as the proportional term of a constant coefficien PID controller. By adjusting the appropriate coefficients, the static and dynamic stability of the aircraft can be greatly enhanced. Due to this excess stability, the center of mass of the vehicle to be moved aft which, in turn, allows for a reduction or complete elimination any reflex in the elevons. This lack of reflects increases the lift-to-drag ratio of the wing compared to a reflexed airfoil.

The simplest angle-of-attack sensor is a [simple low-friction potentiometer attached to a weathervane](https://www.ilmailu.org/forum/index.php?action=dlattach;topic=5147.0;attach=10336). As the weathervane will closely follow the local airflow, it can be used as a reference from which to measure the angle of attack. 

The program is designed to receive 2 PWM inputs from an RC receiver and an analog signal, and outputs two PWM signals for two servos. It is assumed the aircraft is controlled with two elevons. It was written for an Arduino Nano but it should be compatible with other boards. It requires the "PinChageInterrupt" library found here:

  https://www.arduino.cc/reference/en/libraries/pinchangeinterrupt/
  
For previous projects that inspired this work, please see these links:

- http://www.charlesriverrc.org/articles/asfwpp/lelke_activepitch.htm
- https://www.youtube.com/watch?v=JfKrUbJYk74

Schematic of the required circuit:

<img src = "tail_less_circuit_schem.png" width = "90%" height = "90%">

Finally, here is an example of the kind of aircraft this code was writen for:

<img src = "/example_pictures/top_view_res.jpg" width = "30%" height = "30%"> <img src = "/example_pictures/front_view_res.jpg" width = "30%" height = "30%"> <img src = "/example_pictures/side_view_res.jpg" width = "30%" height = "30%">




