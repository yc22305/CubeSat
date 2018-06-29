# CubeSat Attitude Control System
This project aims at a CubeSat attitude control system with thrusters as the accuator. 9-DoF IMU is used to get attitude information, along with pulse modulation algorithm to determine when to activate thrusters. In addition to the control system, ROS (robotic operating system) is utilized for communication with PC or other devices. Remote communication is important for experiments since we can send user commands and get CubeSat status information during its operation, which enhaces safety and convenience. 

In this reporitory, "arduino" folder has files for the real control system, while "Matlab" folder provides simulation programs. To use the complete functions in this project, simply follow the instruction indicated below.
- As for "arduino" (2018/06/28 updated):  
"CubeSate_controller_1D_rosserial" folder has the sketches (code file specific for arduino) providing complete functions for this project. To use the code, several libraries should be set up:
  1. Add all the .zip files to the your arduino libraries. They are opensources available on the Internet.
  2. Set up rosserial libraries for arduino. The tutorial: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
  
- As for Matlab (2018/06/28 updated):  
"pulse_modulator" .m file provides a simulation program for the attitude control system. The stretegy is credited to https://www.sciencedirect.com/science/article/pii/S1270963805000908.

Details of usage are stated in each code file.

#### [NOTE:]  
Other files not mentioned above but included in the reporitory are for testing purposes, or are still under development, even are forgone in this project yet retained. They are not important, but offered for reference.

---
#### [Current progress (2018/06/28 updated): ]
  1-D attitude (yaw) control. Users are able to input, from a remote device, the targeted orientation angle for the CubeSat to track. 

## The Implement of our Control System
### Center: Arduino DUE
Among each version of arduino board, arduino DUE is chosen in this project. DUE indeed has several advantages for our implementation. First, Instead of including <SoftWareSerial.h> to set RX and TX pins, DUE has already defined them in the hardware. Documentation on arduino official website https://store.arduino.cc/arduino-due clearly states the usage of these pins. It is DUE's strength because <SoftWareSerial.h> cannot sustain higher baudrate to transmit data; baudrate of 9600 is probably the maximun to transmit satisfactory data, or data might be contaminated or missed. DUE does not has such problem. Second, DUE has enough memory to include ROS libraries. It is not to say other versions are unacceptable, but ROS libraries may consume so large memory that DUE is undoubtedly a good option.

### MPU9250
MPU9250 is a 9-DoF IMU, including an accelerometer, a gyroscope, and a magnetometer. The data reveived from these three sensors are fused to get the attitude of our CubeSat, througn Mahony Filter. The code is credited to https://github.com/kriswiner/MPU9250. To fit this project, slight modification is made and uploaded in the folder "arduino/CubeSate_controller_1D_rosserial".

- Hardware wiring:  
  <i>MPU9250 <---> Arduino</i>  
   VCC <---> 3.3 V  
   GND <---> GND  
   SCL <---> Arduino SCL (pull high)  
   SDA <---> Arduino SDA (pull high)  
   AD0 <---> 3.3 V  

### LCD (optional)
LCD is equiped on the CubeSat for indicating some information. It is an optional setup in this project because during experiments, remote monitoring is much more crucial. The code is credited to https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library. To fit this project, slight modification is made and uploaded as "LiquidCrystal_I2C_Wire1.zip".

#### [NOTE:]  
Arduino DUE provides two sets of I2C communication pins: "SCL & SDA" and "SCL1 & SDA1". Because "SCL & SDA" is occupied by MPU9250, LCD therefore uses another set. Simply modifying the virtual object "Wire" to "Wire1" (ex: Wire.begin() -> Wire1.begin()) enables "SCL1 & SDA1". "Wire1" is the inherent definition in Arduino DUE.

- Hardware wiring:  
   <i>LCD <---> Arduino</i>  
   VCC <---> 5.0 V  
   GND <---> GND  
   SCL <---> Arduino SCL1 (pull high)  
   SDA <---> Arduino SDA1 (pull high)  


### Bluetooth


### Relay
#### [NOTE:]
1. It is important to set each hardware at the same GND, or the logics will be wrongly determined.
2. Implements in remote devices are described in another reporitory "Arduino_ROS_Communication". 
