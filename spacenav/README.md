# Spacenav Node #
##### Published topics #####
* `spacenav/offset` (geometry_msgs/msg/Vector3)

   Publishes the linear component of the joystick's position. Approximately normalized to a range of -1 to 1.
* `spacenav/rot_offset`(geometry_msgs/msg/Vector3)

   Publishes the angular component of the joystick's position. Approximately normalized to a range of -1 to 1.
* `spacenav/twist` (geometry_msgs/msg/Twist)

   Combines offset and rot_offset into a single message.
* `spacenav/joy` (sensor_msgs/msg/Joy)

   Outputs the spacenav's six degrees of freedom and its buttons as a joystick message.

##### Parameters #####
   sets values to zero when the spacenav is not moving
* `zero_when_static` (boolean, default: true)
* `static_count_threshold` (int, default: 30)

   The number of polls needed to be done before the device is considered "static"
* `static_trans_deadband` (float, default: 0.1)

   sets the translational deadband
* `static_rot_deadband` (float, default: 0.1)

   sets the rotational deadband
* `linear_scale/x` (float, default: 1)
* `linear_scale/y` (float, default: 1)
* `linear_scale/z` (float, default: 1)

   sets the scale of the linear output
* `angular_scale/x` (float, default: 1)
* `angular_scale/y` (float, default: 1)
* `angular_scale/z` (float, default: 1)
   sets the scale of the angular output

## Running the spacenav node ##

Running the node is straightforward
```
$ ros2 run spacenav spacenav_node
```
The node is now publishing to the topics listed above.
