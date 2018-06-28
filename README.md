# CubeSat Attitude Control System
This project aims at a CubeSat attitude control system with thrusters as the accuator. 9-DoF IMU is used to get attitude information, along with pulse modulation algorithm in arduino to determine when to activate thrusters. In addition to the control system, ROS (robotic operating system) is utilized for communication with PC or other devices. Remote communication is important for experiments since we can send user commands and get CubeSat status information during its operation, which enhaces safety and convenience.

In this reporitory, "arduino" folder has files for arduino boards, while "Matlab" folder has simulation programs.

- Current progress: 
  1-D attitude (yaw) control. Users are able to input, from a remote device, the targeted orientation angle for the CubeSat to track. 

## Control System
### Center: Arduino DUE
Among each version of arduino board, arduino DUE is chosen in this project. DUE indeed has several advantages for our implementation. First, Instead of including <SoftWareSerial.h> to set RX and TX pins, DUE has already defined them in the hardware. Documentation on arduino official website https://store.arduino.cc/arduino-due clearly states the usage of these pins. It is DUE's strength because <SoftWareSerial.h> cannot sustain higher baudrate to transmit data; baudrate of 9600 is probably the maximun to transmit satisfactory data, or data might be contaminated or missed. DUE does not has such problem. Second, DUE has enough memory to include ROS libraries. It is not to say other versions are unacceptable, but ROS libraries may consume so large memory that DUE is undoubtedly a good option.

### MPU9250
MPU9250 is a 9-DoF IMU, including an accelerometer, a gyroscope, and a magnetometer. The data reveived from these three sensors are fused to get the attitude of our CubeSat, througn Mahony Filter. Code is credited to https://github.com/kriswiner/MPU9250. To fit this project, slight modification is made and uploaded in "arduino/CubeSate_controller_1D_rosserial/".

- Hardware wiring:
   VCC <---> 3.3 V
   GND <---> GND
   SCL <---> Arduino SCL (pull high)
   SDA <---> Arduino SDA (pull high)
   AD0 <---> 3.3 V

### LCD
### Bluetooth
### Relay
#### [NOTE:]
1. It is important to set each hardware at the same GND, or the logics will be wrongly determined.
2. Comments in "CubeSate_controller_1D_rosserial.ino" shoube be read to understand how to use the control system.
3. Implements in remote devices are described in another reporitory "Arduino_ROS_Communication". 
