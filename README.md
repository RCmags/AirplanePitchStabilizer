# Pitch-axis stabilizer for airplane :airplane:
This Arduino sketch is a servo controller for a tail-less RC airplane that is stabilized about the pitch-axis. If tuned correctly, It may increase the lift-to-drag ratio of an aircraft that otherwise depends solely on passive stability.  

## Concept

__Angle of attack sensor__:
To provide this stability enhancement, the code uses the signal an analog A.O.A. sensor as the proportional term of a constant coefficien PID controller. By adjusting the appropriate coefficients, the static and dynamic stability of the aircraft can be greatly enhanced. Due to this excess stability, the center of mass of the vehicle to be moved aft which, in turn, allows for a reduction or complete elimination any reflex in the elevons. This lack of reflects increases the lift-to-drag ratio of the wing compared to a reflexed airfoil.

The simplest angle-of-attack sensor is a [simple low-friction potentiometer attached to a weathervane](https://www.ilmailu.org/forum/index.php?action=dlattach;topic=5147.0;attach=10336). As the weathervane will closely follow the local airflow, it can be used as a reference from which to measure the angle of attack. 

__Inertial measurement unit (IMU)__: One can maintain the attitude of the aircraft by measuring its change in orientation, and providing an appropriate correction. In this case, we use the angular velocity of a rate gyroscope as the proportional term in a PID controller. The integral term is the angular deflection, which is approximately equal to the angle of attack for small disturbances. 

## How it works
The program is designed to receive 2 PWM inputs from an RC receiver and an analog signal, and outputs two PWM signals for two servos. It is assumed the aircraft is controlled with two elevons. It was written for an Arduino Nano but it should be compatible with other boards. 

## Schematics
Schematic of the required circuit:

__Nano:__ There are two version of the sketch for the arduino nano. Each uses the stabilization concepts discussed above. 

- Angle of attack sensor: This schematic assumes a potentiometer is used as the sensor, but any analog sensor will work. A hall sensor is a low-friction alternative of the potentiometer.

<p align="center"> 
<img src = "/images/diagrams/nano/aoa-sensor/schematic-nano.png" width = "80%" height = "80%">
</p>

- IMU sensor: The code is designed around an MPU6050, but the general structure will work for any IMU.
<p align="center"> 
<img src = "/images/diagrams/nano/imu/schematic-imu.png" width = "80%" height = "80%">
</p>

__DigiSpark ATTiny85__: The version of the sketch does not have the option for an IMU. Rather, it serves as a lighter alternative of the Nano version for an angle of attack sensor. 
<p align="center"> 
<img src = "/images/diagrams/attiny/schematic-attiny.png" width = "80%" height = "80%">
</p>

## Dependencies
The IMU version of the sketch requires the following libraries to compile:

- [basicMPU6050](https://github.com/RCmags/basicMPU6050)
- [imuFilter](https://github.com/RCmags/imuFilter)

## References
For previous projects that inspired this work, see these pages:

- [Actively stabilized pitch axis](http://www.charlesriverrc.org/articles/asfwpp/lelke_activepitch.htm)
- [Actively stabilized glider](https://www.youtube.com/watch?v=JfKrUbJYk74)

## Examples
Here is the original airplane the code was writen for:

<p align="center">
<img src = "/images/example/top_view_res.jpg" width = "30%" height = "30%">
<img src = "/images/example/front_view_res.jpg" width = "30%" height = "30%"> 
<img src = "/images/example/side_view_res.jpg" width = "30%" height = "30%">
</p>
