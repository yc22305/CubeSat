# ROS Package for Communication with our CubeSat 
This package implements ROS nodes on PC to communicate with the Arduino board adopted by our CubeSat through bluetooth or serials. The nodes' function is to receive the information of CubeSat status such as attitude, and send user commands like the angle for tracking.

To fully use the nodes, `rviz` should be installed. The tutorial: http://wiki.ros.org/rviz/UserGuide.
## Topic nodes
### debug_node (subscriber) 
command:
```
rosrun serial_communication debug_node
```

This node is used to receive the following information:
1. Whether the relays are powered on. Default: OFF
2. The current desired tracking angle is. Default: 0
3. The current status of thruster (which thruster is activated)

### tf_displayer (subscriber)
command:
```
rosrun serial_communication tf_displayer
```

This node is used to receive attitude information and publish it to `rviz` for graphic display. After running this node, command `rosrun rviz rviz` to conduct graphic display, or "tf_displayer" will be stuck. You could refer to this tutorial http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes to further understand the code. In addition, the attitude information will also be shown on the terminal.

#### [NOTE:]
It is normal to get error message at first because tf_displayer node probably still have not receive information from Arduino.

## Service nodes
### set_desiredValue (client)
command:
```
rosrun serial_communication set_desiredValue
```

This node is used to set a desired angle for the CubeSat to track. After the command, the terminal will ask you to enter an angle between -180 and 180; if the value is out of the range, the node will exit. The default value is 0.

Our CubeSat body frame is referred to the frame with x-axis toward to Earth north, y-axis toward to Earth east, and z-axis toward the ground.

### power_thruster (client)
command:
```
rosrun serial_communication power_thruster
```

This node is used to set whether to power on the thrusters. After the command, the terminal will ask you to enter `Y` or `N` to determine the power status. The default is OFF.

#### [NOTE:]
For every service nodes, it is normal to get error message of not getting feedback from arduino because the implements of rosserial_arduino still have not had the ability to feedback information from callback functions in arduino. You will see a yellow message telling you to ignore the error.
