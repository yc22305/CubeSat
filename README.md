# CubeSat Attitude Control System
This project aims at a CubeSat attitude control system with thrusters as the accuator. 9-DoF IMU is used to get attitude information, along with pulse modulation algorithm to determine when to activate thrusters. In addition to the control system, ROS (robotic operating system) is utilized for communication with PC or other devices remotely. Remote communication is important for experiments since we can send user commands and get status information during the CubeSat's operation, which enhaces safety and convenience. 

In this reporitory, "arduino" folder has files for the real control system, while "Matlab" folder provides simulation programs. To use the complete functions in this project, simply follow the instruction indicated below.
- As for "arduino" --- (2018/06/28 updated):  
"CubeSate_controller_1D_rosserial" folder has the sketches (code file specific for arduino) providing complete functions for this project. To use the code, several libraries should be set up:
  1. Add all the .zip files to the your arduino libraries. They are opensources available on the Internet.
  2. Set up rosserial libraries for arduino. The tutorial: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
  
- As for Matlab --- (2018/06/28 updated):  
"pulse_modulator" .m file provides a simulation program for the attitude control system. The stretegy is credited to https://www.sciencedirect.com/science/article/pii/S1270963805000908.

Details of function usage and parameter setting are stated in each code file.

#### [NOTE:]  
Other files not mentioned above but included in the reporitory are for testing purposes, or are still under development, even are forgone in this project yet retained. They are not important, but offered for reference.

---
#### [Current progress (2018/06/28 updated):]
  1-D attitude (yaw) control. Users are able to input, from a remote device, the targeted orientation angle for the CubeSat to track. 

## The Implement of our Control System
### Center: Arduino DUE
Among each version of arduino board, arduino DUE is chosen in this project. DUE indeed has several advantages for our implementation. **First**, Instead of including <SoftWareSerial.h> to set RX and TX pins, DUE has already defined them in the hardware. Documentation on arduino official website https://store.arduino.cc/arduino-due clearly states the usage of these pins. It is DUE's strength because <SoftWareSerial.h> cannot sustain higher baudrate to transmit data; baudrate of 9600 is probably the maximun to transmit satisfactory data, or data might be contaminated or missed. DUE does not has such problem. **Second**, DUE has enough memory to include ROS libraries. It is not to say other versions are unacceptable, but ROS libraries may consume so large memory that DUE is undoubtedly a good option.

### MPU9250
MPU9250 is a 9-DoF IMU, including an accelerometer, a gyroscope, and a magnetometer. The data reveived from these three sensors are fused to get the attitude of our CubeSat, througn Mahony Filter. The code is credited to https://github.com/kriswiner/MPU9250. To fit this project, slight modification is made and uploaded in the folder "arduino/CubeSate_controller_1D_rosserial". The following is hardware wiring:  
- *MPU9250 <---> Arduino*  
   VCC <---> 3.3 V  
   GND <---> GND  
   SCL <---> Arduino SCL (pull high)  
   SDA <---> Arduino SDA (pull high)  
   AD0 <---> 3.3 V  

### LCD (optional)
LCD is equiped on the CubeSat for indicating some information. It is an optional setup in this project because during experiments, remote monitoring is much more crucial. The code is credited to https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library. To fit this project, slight modification is made and uploaded as "LiquidCrystal_I2C_Wire1.zip". The following is hardware wiring:
 
- *LCD <---> Arduino*  
   VCC <---> 5.0 V  
   GND <---> GND  
   SCL <---> Arduino SCL1 (pull high)  
   SDA <---> Arduino SDA1 (pull high)  

#### [NOTE:]  
Arduino DUE provides two sets of I2C communication pins: "SCL & SDA" and "SCL1 & SDA1". Because "SCL & SDA" is occupied by MPU9250, LCD therefore uses another set. Simply modifying the virtual object "Wire" to "Wire1" (ex: Wire.begin() -> Wire1.begin()) enables "SCL1 & SDA1". "Wire1" is an inherent definition in Arduino DUE. By the way, do not forget to modify the header file. 

### Bluetooth HC-05
HC-05 is used for remote communication. Steps to establish this communication are: **First**, baudrate of HC-05 should be set properly according to our needs; 57600 is set in this project. Too high or too low baudrate may lead to bad quility information, so trial and error is imperitive. **Second**, a virtual port is necessary to be created to revice the data from HC-05. **Third**, "serial_node.py", a program provided by "rosserial" libraries, is activated to link the CubeSat and our PC.

#### \<Step1\>: Set the Baudrate:
To set the baudrate, we need to switch HC-05 into **"AT mode"**. "AT mode" is a firmware for users to set up parameters in some devices, which are not limited to HC-05, but others like ESP8266-01 (Wifi module) also adopt this firmware. In AT mode, we are able to use several commands to change the defult parameters, including baudrate; however, there are numerious types and versions of AT mode depending on the devices, despite the similarity. Therefore, we need to find the corresponding AT command list for particular devices. Fortunately, all HC-05s seem to use the same AT commands listed here: https://www.itead.cc/wiki/Serial_Port_Bluetooth_Module_(Master/Slave)_:_HC-05#3._Get_firmware_version; if not, it might be version descrepency.

Entering AT mode is nothing more than two steps:  
1. Press the button (only one button) on HC-05, hold it, before power HC-05.  
2. Power HC-05, and then release the button.  

You should find that the red LED is flashing about every two seconds, which indicates AT mode. The power supply is 5V.

Bunches of methods for communicating with HC-05 in AT mode could be found on the Internet; here, arduino DUE is set as USB-to-TTL for us to send AT command through serial monitor provided in arduino IDE. Hardware wiring should follow:

- *HC-05 <---> Arduino*  
   VCC <---> 5.0 V  
   GND <---> GND
   GND <---> RESET
   RX <---> RX0 (pin 0)  
   TX <---> TX0 (pin 1)  
  
RX0 and TX0 in arduino DUE are connected to the corresponding pins of its USB-to-TTL Serial chip, so wiring in this way could bypass information from Serial port (programming port) directly to RX0 and TX0 (where HC-05 is connected) rather than SAM3X chip (the chip for calculating) in DUE. Be aware that RX0 and TX0 is named relatively to arduino DUE; that means, connecting RX with RX (TX with TX) leads to "fuse" HC-05 into DUE, as HC-05 is a part of DUE other than an extra device. The RESET pin on DUE must connect to GND for disabling SAM3X chip.

After entering AT mode, we open the Serial monitor provided by arduino IDE to send commands. For example, typing "AT+UART=57600,1,0" will modify the baudrate to 57600, the stop bit to 1, and the parity to 0. Commands and their usage are listed in the website mentioned. Remember to set Serial monitor baudrate to 38400, which is particular to message transmitting in AT mode and cannot be modified. 

Finally, we switch HC-05 back to normal mode and reset the hardware wiring. Simply repowering HC-05 without pressing the button activates normal mode, and wiring is set as the following:

- *HC-05 <---> Arduino*  
   VCC <---> 5.0 V  
   GND <---> GND  
   RX <---> TX0 (pin 1)  
   TX <---> RX0 (pin 0)  

Actually, which TX and RX on DUE are used is up to you, but TX0 and RX0 are defult in "rosserial_arduino" package. How to modify the transmitting pins will be instructed in the "rosserial" section.

#### \<Step2:\> Create the corresponding virtual port:

### Relay
#### [NOTE:]
1. It is important to set each hardware at the same GND, or the logics will be wrongly determined.
2. Implements in remote devices are described in my another reporitory "Arduino_ROS_Communication".
3. ROS tutorial if needed: http://wiki.ros.org/ROS/Tutorials
