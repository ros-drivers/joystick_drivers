# Testing procedures for the WiiMote package

### Python compatiblity test script ###
After building the package the first thing one should run is the python test script

The script will look for compatible Wiimotes. The package is **not** compatible with the newer Wiimotes. The compatible Wiimotes are the **Nintendo RVL-CNT-01** and **Nintendo RVL-WBC-01**.

```
$ python wiimote/scripts/wiimote_test.py
Looking for compatible devices
Incompatible controller:
 CC:FB:65:E6:62:1D - Nintendo RVL-CNT-01-TR
Compatible controller:
 00:17:AB:37:FF:5E - Nintendo RVL-CNT-01
```

In the above example two wiimotes were deteced but only one is compatible with the wiimote package.

---

### Running the wiimote nodes ###
The package consists of two primary nodes:
>wiimote_node
wiimote_node.py

The C++ implementation was design with focus on reduced resource consumption and is recommended over the python variant.

---

#### Wiimote_node ####

Running the wiimote_node is straight forward, ensure bluetooth is enabled on your machine and put the Wiimote in discoverable mode (press 1+2)

```
$ rosrun wiimote wiimote_node
[ INFO] [1498662258.081403347]: * * * Pairing
[ INFO] [1498662258.081474540]: Allow all joy sticks to remain at center position until calibrated.
[ INFO] [1498662258.081506464]: Put Wiimote in discoverable mode now (press 1+2)...
[ INFO] [1498662258.081540133]: Timeout in about 5 seconds if not paired.
[ INFO] [1498662835.763122784]: Collecting additional calibration data; keep wiimote stationary...
[ INFO] [1498662836.780853688]: Wiimote is Paired
```
The wiimote is now paired and is publisihing to several different topics. Listing the Ros topics should yield something similar to:

```
$ rostopic list
/imu/data
/imu/is_calibrated
/joy
/joy/set_feedback
/rosout
/rosout_agg
/wiimote/state
```
##### Possible Errors #####

>If the Wiimote fails to connect try pressing the red sync button on the back of the remote and run the node again. The script could also be timing out before it is able to find the Wiimote aswell. If this is suspected try running the node with the following parameters:
`
$ rosrun wiimote wiimote_node _pair_timeout:=-1
`
>This will make the script search indefinitly until closed.

To see if the Wiimote is working properly echo the /wiimote/state topic
```
$rostopic echo /wiimote/state
---
header:
  seq: 485
  stamp:
    secs: 1498664301
    nsecs: 333370601
  frame_id: ''
angular_velocity_zeroed:
  x: 0.0
  y: 0.0
  z: 0.0
angular_velocity_raw:
  x: 0.0
  y: 0.0
  z: 0.0
angular_velocity_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration_zeroed:
  x: 1.63444166667
  y: 0.338160344828
  z: 9.80665
linear_acceleration_raw:
  x: 144.0
  y: 140.0
  z: 161.0
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.7766074343434308, 0.0, 0.0, 0.0, 1.1203354696969712]
nunchuk_acceleration_zeroed:
  x: 0.0
  y: 0.0
  z: 0.0
nunchuk_acceleration_raw:
  x: 0.0
  y: 0.0
  z: 0.0
nunchuk_joystick_zeroed: [0.0, 0.0]
nunchuk_joystick_raw: [0.0, 0.0]
buttons: [False, False, False, False, False, False, False, False, False, False, False]
nunchuk_buttons: [False, False]
LEDs: [False, False, False, False]
rumble: False
ir_tracking:
  -
    x: -1.0
    y: -1.0
    ir_size: -1
  -
    x: -1.0
    y: -1.0
    ir_size: -1
  -
    x: -1.0
    y: -1.0
    ir_size: -1
  -
    x: -1.0
    y: -1.0
    ir_size: -1
raw_battery: 155.0
percent_battery: 74.5192337036
zeroing_time:
  secs: 1498662949
  nsecs: 612081279
errors: 0
---
```

Test it out! Move the Wiimote and see the corresponding values change on the terminal.

---
### Wiimote_node.py ###
The process for the python variant is similar to the C++ version. Put the Wiimote in discoverable mode (press 1+2) and run the script.

```
$ rosrun wiimote wiimote_node.py
Press buttons 1 and 2 together to pair (within 6 seconds).
    (If no blinking lights, press power button for ~3 seconds.)
```
Echo the output onto the screen using the same `$ rosparam echo /wiimote/state` used above.


### Topics ###
##### Subscribed topics #####
* set_feedback (sensor_msgs/JoyFeedbackArray)
*Topic where ROS clients control the Wiimote's leds and rumble (vibrator) facility.* 

##### Published Topics #####
* joy (joy/Joy) 

   *Topic on which Wiimote accelerometer, gyro, and button data are published. Axes are: linear AccelerationX/Y/Z, followed by angular velocityX/Y/Z (a.k.a. Roll, Pitch, Yaw, a.k.a. Phi, Theta, Psi). The values are corrected to be near zero at rest. For raw values, use the State message. The Wiimote buttons are reported in the same order as in the State message*  

* imu/data (sensor_msgs/Imu) 

   *Topic on which Wiimote gyro and accelerometer data are sent out.*  

* wiimote/state (wiimote/State)

   *Topic for comprehensive information about Wiimote state.*  

* wiimote/nunchuk (joy/Joy)

   *Topic on which data from an attached Nunchuk are sent out, including the joystick, accelerometer and button data. Axes are: Joystick X/Y, Acceleration X/Y/Z. The buttons are Z then C. Note the spelling of the topic name does not include a 'c'.*  

* wiimote/classic (joy/Joy)

   *Topic on which data for a Wiimote classic attachment are sent out (Untested by Willow Garage).*  

* imu/is_calibrated (std_msgs/Bool)

   *Latched topic for learning the most recent Wiimote calibration result.*  


