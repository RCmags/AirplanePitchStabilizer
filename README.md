# Pitch-axis stabilizer for airplane
This Arduino sketch is a servo controller for a tail-less RC airplane that is stabilized about the pitch-axis. If tuned correctly, It may increase the lift-to-drag ratio of an aircraft that otherwise depends solely on passive stability.  

## Concept

__Trailing edge flaps__:
To provide this stability enhancement, the code uses the signal an analog A.O.A. sensor as the proportional term of a constant coefficien PID controller. By adjusting the appropriate coefficients, the static and dynamic stability of the aircraft can be greatly enhanced. Due to this excess stability, the center of mass of the vehicle to be moved aft which, in turn, allows for a reduction or complete elimination any reflex in the elevons. This lack of reflects increases the lift-to-drag ratio of the wing compared to a reflexed airfoil.

The simplest angle-of-attack sensor is a [simple low-friction potentiometer attached to a weathervane](https://www.ilmailu.org/forum/index.php?action=dlattach;topic=5147.0;attach=10336). As the weathervane will closely follow the local airflow, it can be used as a reference from which to measure the angle of attack. 

__Weight shift__:

## How it works
The program is designed to receive 2 PWM inputs from an RC receiver and an analog signal, and outputs two PWM signals for two servos. It is assumed the aircraft is controlled with two elevons. It was written for an Arduino Nano but it should be compatible with other boards. 
 
## References
For previous projects that inspired this work, see these pages:

- [Actively stabilized pitch axis](http://www.charlesriverrc.org/articles/asfwpp/lelke_activepitch.htm)
- [Actively stabilized glider](https://www.youtube.com/watch?v=JfKrUbJYk74)

## Schematics
Schematic of the required circuit:

__Nano:__

- Angle of attack sensor:
<p align="center"> 
<img src = "/images/diagrams/nano/aoa-sensor/schematic-nano.png" width = "80%" height = "80%">
</p>

- IMU sensor: 
<p align="center"> 
<img src = "/images/diagrams/nano/imu/schematic-imu.png" width = "80%" height = "80%">
</p>

__DigiSpark ATTiny85__:
<p align="center"> 
<img src = "/images/diagrams/attiny/schematic-attiny.png" width = "80%" height = "80%">
</p>

## Examples
Here is the original airplane the code was writen for:

<p align="center">
<img src = "/images/example/top_view_res.jpg" width = "30%" height = "30%">
<img src = "/images/example/front_view_res.jpg" width = "30%" height = "30%"> 
<img src = "/images/example/side_view_res.jpg" width = "30%" height = "30%">
</p>



