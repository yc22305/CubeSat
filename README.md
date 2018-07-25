# CubeSat Attitude Control System
This project aims at a CubeSat attitude control system with thrusters as the accuator. 9-DoF IMU is used to get attitude information, along with pulse modulation algorithm to determine when to activate thrusters. In addition to the control system, ROS (robotic operating system) is utilized for remote communication bewteen the CubeSat and our PC. This remote communication system enables users to send commands and get information during the CubeSat's operation, which enhaces safety and convenience. 

In this reporitory, "arduino" folder has files for the real control system, while "Matlab" folder provides simulation programs. **To use the complete functions in this project, simply follow the instruction indicated below**:  

- As for *arduino* --- (2018/06/28 updated):  
`CubeSat_controller_1D_rosserial` folder has the sketches (code files specific for arduino) providing complete functions for this project. To use the code, several libraries should be set up:
  1. Add all the `.zip` files to the your arduino libraries.
  2. Set up `rosserial libraries` with reference to [the tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup). **Addtionally, please refer to "Rosserial: [Setting up in arduino code](https://github.com/yc22305/CubeSat#setting-up-in-arduino-code)" for possible further setting.**
  3. Generate the header files of costom ROS message types (defined in the package `serial_srvs`, which is uploaded in this reporitory under `/CubeSat`) into arduino libraries. **Please refer to "Rosserial: [Define a custom ROS message type in arduino](https://github.com/yc22305/CubeSat#define-a-custom-ros-message-type-in-arduino)"**.
  
- As for *Matlab* --- (2018/06/28 updated):  
`pulse_modulator` is a simulation program for the attitude control system. The stretegy is credited to [this paper]( https://www.sciencedirect.com/science/article/pii/S1270963805000908).

- As for *PC_terminal* --- (2018/07/25 updated):  
`serial_communication` is a ROS package for PC to communicate with our CubeSat through bluetooth or serials.

Details of function usage and parameter setting are stated by comments in each code file.

##### [NOTE:]
1. ROS is operating under Linux envionment, so the PC connected to arduino boards must be the ROS master running in Linux. In this project, `ubuntu 16.04` is used.
2. [ROS tutorial](http://wiki.ros.org/ROS/Tutorials) if needed. Being familar with ROS is a prerequisite before using rosserial libraries.

#### [Current progress (2018/06/28 updated):]
  1-D attitude (yaw) control. Users are able to input, from a remote device, the target orientation angle for the CubeSat to track. 

---
## The Implement of our Control System
### Center: Arduino DUE
Among each version of arduino board, arduino DUE is chosen in this project. DUE indeed has several advantages for our implementation. **First**, instead of including <SoftWareSerial.h> to set RX and TX pins, DUE has already defined them in the hardware. Documentation on [arduino official website](https://store.arduino.cc/arduino-due) clearly states the usage of these pins. It is DUE's strength because <SoftWareSerial.h> cannot sustain higher baudrate to transmit data; baudrate of 9600 is probably the maximun to transmit satisfactory data, or data might be contaminated or missed. DUE does not has such problem. **Second**, DUE has enough memory to include ROS libraries. Other versions of arduino board might be acceptable, but ROS libraries may consume so large memory that DUE is undoubtedly a good option.

### MPU9250
MPU9250 is a 9-DoF IMU, including an accelerometer, a gyroscope, and a magnetometer. The data reveived from these three sensors are fused to get the attitude of our CubeSat, througn Mahony Filter. The code is credited to https://github.com/kriswiner/MPU9250. To fit this project, slight modification is made and uploaded in the folder "arduino/CubeSat_controller_1D_rosserial". The following is hardware wiring:

- *MPU9250 <---> Arduino*  
   VCC <---> 3.3 V  
   GND <---> GND  
   SCL <---> Arduino SCL (pull high)  
   SDA <---> Arduino SDA (pull high)  
   AD0 <---> 3.3 V  

Our CubeSat attitude is referred to the frame with x-axis toward to Earth north, y-axis toward to Earth east, and z-axis toward the ground. It is important to input the vectors (acceleration, etc.) **relative to the reference frame** into Mahony filter, or the filter output will be messed up.

### LCD (optional)
LCD is equiped on the CubeSat for indicating some information. It is an optional setup in this project because during experiments, remote monitoring is much more crucial. The code is credited to https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library. To fit this project, slight modification is made and uploaded as "LiquidCrystal_I2C_Wire1.zip". The following is hardware wiring:
 
- *LCD <---> Arduino*  
   VCC <---> 5.0 V  
   GND <---> GND  
   SCL <---> Arduino SCL1 (pull high)  
   SDA <---> Arduino SDA1 (pull high)  

##### [NOTE:]  
Arduino DUE provides two sets of I2C communication pins: "SCL & SDA" and "SCL1 & SDA1". Because "SCL & SDA" is occupied by MPU9250, LCD therefore uses another set. Simply modifying the virtual object "Wire" to "Wire1" (ex: Wire.begin() -> Wire1.begin()) enables "SCL1 & SDA1". "Wire1" is an inherent definition in Arduino DUE. By the way, do not forget to modify the header file. 

### Bluetooth HC-05
HC-05 is used for remote communication. Steps to establish this communication are: **First**, baudrate of HC-05 should be set properly according to our needs. In this project, 115200 baudrate is set, along with publishing ROS messages in 20 Hz. Too high or too low baudrate may corrupt information, so trial and error is imperitive. **Second**, a virtual port is necessary to be created to revice the data from HC-05. **Third**, "serial_node.py", a program provided by rosserial libraries, is activated to link the CubeSat and our PC.

#### \<Step1\>: Set the Baudrate:
To set the baudrate, we need to switch HC-05 into **"AT mode"**. "AT mode" is a firmware for users to set up parameters in some devices, which are not limited to HC-05, but others like ESP8266-01 (Wifi module) also adopt this firmware. In AT mode, we are able to use several commands to change the defult parameters, including baudrate; however, there are numerious types and versions of AT mode depending on the devices, despite the similarity. Therefore, we need to find the corresponding AT command list for particular devices. Fortunately, all HC-05s seem to use the same AT commands listed [here]( https://www.itead.cc/wiki/Serial_Port_Bluetooth_Module_(Master/Slave)_:_HC-05#3._Get_firmware_version); if not, it might be version descrepency.

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
  
RX0 and TX0 in arduino DUE are connected to the corresponding pins of its USB-to-TTL Serial chip, so wiring in this way could bypass information from Serial port (programming port) directly to RX0 and TX0 without passing SAM3X chip (the chip for main operation in DUE). Be aware that RX0 and TX0 are named relatively to arduino DUE; that means, connecting RX with RX (and TX with TX) will "fuse" HC-05 into DUE, as HC-05 is a part of DUE other than an extra device. The RESET pin on DUE must connect to GND for disabling SAM3X chip.

After entering AT mode, we open the Serial monitor provided by arduino IDE to send commands. For example, typing `AT+UART=115200,1,0` will modify the baudrate to 115200, the stop bit to 1, and the parity to 0. Commands and their usage are listed in the website mentioned. Remember to set Serial monitor baudrate to 38400, which is particular to message transmitting in AT mode and cannot be modified. 

Finally, switch HC-05 back to normal mode and reset the hardware wiring. Repowering HC-05 without pressing the button activates normal mode, and wiring is set as the following:

- *HC-05 <---> Arduino*  
   VCC <---> 5.0 V  
   GND <---> GND  
   RX <---> TX0 (pin 1)  
   TX <---> RX0 (pin 0)  

Actually, which TX and RX on DUE are used is up to you (you only need some software setting), but TX0 and RX0 are defult in "rosserial_arduino" package. **In this project, the implement is to set TX1 and RX1 connected to HC-05**. The reason of using TX1 and RX1 instead of TX0 and RX0 is that occupying the latter UART will disable Arduino Serial Monitor (in fact, you could still open the monitor, but this will confuse your PC about which place to send the data to: Serial Monitor or ROS master). **How to modify these UART pins will be instructed in "Rosserial: [Change the serial port for transmitting ROS message](https://github.com/yc22305/CubeSat#change-the-serial-port-for-transmitting-ros-message)"**.

#### \<Step2:\> Create the corresponding virtual port:
A virtual port is created to be bound with our HC-05. Before that, tools for bluetooth management are needed to be installed:
- console tool
```
sudo apt-get install bluetooth bluez bluez-tools rfkill
```
After the installation, open a new terminal (Ctrl+T) and command `bluetoothctl` to use this bluetooth management tool from our shell. Commands such as how to pair bluetooth could be referred to [the website](https://wiki.archlinux.org/index.php/bluetooth).
- UI interface
```
sudo apt-get install blueman
```
A tutorial of using this interface could be referred to [this page](https://www.maketecheasier.com/setup-bluetooth-in-linux/).

After pairing bluetooth, use the fllowing command to create a virtual port:
```
sudo rfcomm bind <port name> <device's address> [channel]
```
`port name` is the virtual port bound with the our `device's address`, and `channel` is the designated channel number. My example is:
```
sudo rfcomm bind rfcomm0 98:D3:31:FC:26:44
```
`port name` must be named as `rfcomm(N)` like rfcomm0, rfcomm1, etc.. If not named in that way, `port name` will be autimatically named as `rfcomm(N)` instead of what we type. `device's address` could be easily found through bluetooth management tools. The argument of 'channel' could be ignored, and the default value is 1. Documentation of `rfcomm` command can be referred to [here](https://www.systutorials.com/docs/linux/man/1-rfcomm/).

Now, in our example, you should find a new file named of `rfcomm0` in `/dev` folder. The path `/dev/rfcomm0` will be used in the next step.

##### [NOTE:]
If you are using a virtual machine such as VMware, every time powering off it will delete the virtual port we create. Thus, we need to create it again in the next time when we would like to use it. I am not sure if the this phenomenon occurs in a real OS.

#### \<Step3:\> Establish the link between the CubeSat and PC:
The link is established in PC terminal. Fisrt, command:
```
roscore
```
Always remember to run `roscore` before running any nodes based on ROS.
Second, command
```
rosrun rosserial_python serial_node.py _port:=/dev/rfcomm0 _baud:=115200
```
`_port:=` is appointed to our virtual port corresponding to HC-05, and `_baud:=` is to set the baudrate. The default baudrate is 57600.

If the arduino board successfully publishes and subsribes ROS messages in loops, you will see messages of setting up in the terminal running `serial_node.py`. Any faults leading to time delay (eg: bad quality of hardwares may cause another device to wait for messages, resulting in time delay) may disconnect the link since the PC terminal's waiting is timed out.

### Relay

##### [NOTE:]
1. It is important to set each hardware at the same GND, or the logics will be wrongly determined.

---
## The Implement of our Communication System
### Rosserial
"rosserial" is a good library for communication between devices based on ROS. We focus on "rosserial_arduino" here. [The tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials) provides several examples; however, I recommand to see the examples offered in arduino IDE (example codes will exsit in arduino IDE after rosserial_arduino libraries are installed) because there are more code sources than the tutorial has, including newest examples for "service server" and "server client".

#### Setting up in arduino code
Different arduino boards may need different initial setup in sketches for the use of rosserial. In **arduino DUE**, two macros should be defined before including <ros.h>: `#define _SAM3XA_` and `#define USE_USBCON`. `_SAM3XA_` is for arduino DUE hardware setting, while `USE_USBCON` is for serial communication in each arduino version except Leonardo. As for other arduino boards, what macros should be defined might be indicated in `ros_lib/ArduinoHardware.h`.

Another setting is about baudrate. `nh.getHardware() -> setBaud(115200)` function makes arduino send and receive data in 115200 baudrate. Without the use of this function, the default baudrate is 57600.

#### Define a custom ROS message type in arduino
After a custom ROS message package is created and complied in your Linux, several steps should be followed to generate the corresponding header file in arduino libraries:
1. Delete the whole `ros_lib` installed in your arduino libraries.
2. run the command `rosrun rosserial_arduino make_libraries.py .`

I have created a package `serial_srvs` and uploded it in this reporitory. Download it and move it to your catkin workspace. Then, follow the above steps and check `ros_lib` folder in arduino libraries for whether `serial_srvs` package is successfully created.

##### [NOTE:]
1. If you fail to create costom packages in `ros_lib`, try to power off the PC (or the virtual machine), restart it, and follow the steps above again.  
2. You could refer to [the page](http://wiki.ros.org/rosserial/Tutorials/Adding%20Other%20Messages) for information about adding message types. "rosserial_arduino" itselt is a "rosserial_client" package, and we would like to retain the functions specific to rosserial_arduino, so just replace rosserial_client with rosserial_arduino.

#### Change the serial port for transmitting ROS message
As mentioned in the fisrt step of "Bluetooth HC-05" section, we are able to wire HC-05 in other way rather than RX0 and TX0. In default, every ROS message will be transmitted through "Serial" in arduino, which is the UART of RX0 & TX0, so HC-05 must be connected to these pin for getting ROS messages. To change the UART, modification in `ros_lib/ArduinoHardware.h` is needed.

The [modified "ArduinoHardware.h"](https://github.com/yc22305/CubeSat/blob/master/ArduinoHardware.h) is uploaded in this reporitory under `/CubeSat`. I add a macro `USE_SERIAL_ONE` for users to decide using "Serial" or "Serial1" in arduino; the code line is around **line 76**. It is a very simple modification so you could easily take a look into the code and make yor own changes.

##### [NOTE:]
Another method to specify the serial port is discussed [here](https://answers.ros.org/question/198247/how-to-change-the-serial-port-in-the-rosserial-lib-for-the-arduino-side/). The method could be concluded as defining a new class for costom usage. However, I found some functions only support the class `ArduinoHardware` (a class provided in the rosserial_arduino package, **NOT the one we CREATE!**), such as `tf::TransformBroadcaster::init()`. That is, a new class definition may lead to failure of using such functions. This might be a negligence of rosserial development.

##### [NOTE:]
1. Implements in remote devices are described in my another reporitory "Arduino_ROS_Communication".
