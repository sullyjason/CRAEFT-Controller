# CRAEFT Controller 

Haptic controller project page

* [STL_files](STL_files) : 3D printing models
* [PCB](PCB): PCB design files (KiCAD 8.0)
* [Test_code](Test_code): calibration and code to test proper controller assembly
* [CRAEFT_controller_fw](CRAEFT_controller_fw): controller code

## Description

<img src="./Images/Instructional/craeft_controller_diagram.png" alt="diagram of the craeft controller" width="90%"/>




## How to use 
* Plug in USB cables A and B as shown. A red LED will indicate that the controller is receiving power

* Optionally, if using a Meta Quest 3, insert the right hand Quest controller into the controller adapter. 



<img src="./Images/Instructional/step1.png" alt= "Inserting the quest controller" width="35%"/> 
<img src="./Images/Instructional/step2.png" alt= "plugging in the USB cables" width="47%"/>

* Open Arduino IDE, Serial port at any baudrate, line termination etc.


* Use the serial monitor to: 
    * Send commands to the controller 
    * Receive information from the controller at 100Hz: the format is

```
    <mode> <target object angle> <PID set Angle> <light_sensor_value> <current force> <stiffness> <thumbstick Y> <thumbstick X>
```
* Mode:
    * Mode 0 : DISABLED MODE
    * Mode 1 : PINCHING MODE - force control
    * Mode 2 : TOUCHING MODE - position

**List of commands**

The following commands can be sent to the controller to change its opearting mode, the angle of the servo arm, and the haptic vibration pattern.

|Command| Acceptable values|Definition|Example|
|:---:|:---:|:---:|:---:|
| M | T, P, D | change the operating mode `op_mode` | MT : puts the controller in TOUCHING MODE |
| A |0 to 50 |update the desired angle of the servo arm `setAngle` |A20 : puts set angle at 20 degrees|
| O |0 to 50 |update the angle at which finger collides with object `objectAngle` |O25 : puts object angle at 30 degrees|
|F |0 to 15000|update haptic vibration frequency `haptic_updateFrequency`| F800 : sets the haptic vibration to 800Hz|
|P|0 to 150|update the haptic vibration amplitude (strength) `haptic_amp`  | P100 : sets the haptic vibration amplitude to 100|
| L | 0 or 1 | allows controller to switch to pinching mode if the light sensor is covered | L1 : activates this function |


**Servo arm range** 

Illustration of what is meant by the previously mentioned angles. The maximum angle, 50 by default, can be configured if needed by changing the global variable `SERVO_ANGLE_MAX`
(such as for users with smaller hands).

<img src="./Images/Instructional/AngleDefinition.png" width="60%" />

## System block diagram
<img src="./Images/Instructional/blockdiagrampng.png" width="80%" />
