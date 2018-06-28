# CubeSat Attitude Control System
This project uses arduino DUE board to implement a CubeSat attitude control system with thrusters as the accuator.
MPU9250 is the IMU to get attitude information, along with pulse modulation to determine when to activate thrusters. 
In addition to the control system, ROS (robotic operating system) is utilized for communication with PC or other devices.
Remote communication is important for experiments since we can send user commands and get CubeSat status information during its operation, 
which enhaces safety and convenience.

## Hardwares
It is important to set each hardware at the same GND, or the logics will be wrongly determined. A breadborad power supply is used 
to provide steady voltage.

### MPU9250
MPU9250 is a 9-DoF IMU, including an accelerometer, a gyroscope, and a magnetometer.
The data reveived from these three sensors are fused to get the attitude of our CubeSat, througn Mahony Filter.
Code is credited to https://github.com/kriswiner/MPU9250. Slight modification is made and uploaded in 

1. Wiring
   VCC <---> 3.3 V
2.

### LCD

