# Spacenav Nodes #
## spacenav_node
##### Published topics
* `spacenav/offset` (geometry_msgs/Vector3) 
   
   Publishes the linear component of the joystick's position. Approximately normalized to a range of -1 to 1. 
* `spacenav/rot_offset`(geometry_msgs/Vector3)

   Publishes the angular component of the joystick's position. Approximately normalized to a range of -1 to 1. 
* `spacenav/twist` (geometry_msgs/Twist)

   Combines offset and rot_offset into a single message. 
* `spacenav/joy` (sensor_msgs/Joy)
   
   Outputs the spacenav's six degrees of freedom and its buttons as a joystick message. 
##### Parameters
* `zero_when_static`


   sets values to zero when the spacenav is not moving
* `static_count_threshold`


   The number of polls needed to be done before the device is considered "static"
* `static_trans_deadband`

   sets the translational deadband
* `static_rot_deadband`
   
   sets the rotational deadband
* `linear_scale`
   
   sets the scale of the linear output
   
   takes a vector (x, y, z)
* `angular_scale`
   
   sets the scale of the angular output
   
   takes a vector (x, y, z)

The `linear_scale` and `angular_scale` parameters take vector arguements.
The easiest way to set these parameters is with a launch file like so:
```
<launch>
  <node pkg="spacenav_node" type="spacenav_node" name="$(anon spacenav_node)" output="screen">
    <rosparam param="linear_scale">[.25, .25, .25]</rosparam>
    <rosparam param="angular_scale">[.5, .5, .5]</rosparam>
  </node>
</launch>
```

### Running the spacenav_node ###

Running the node is straightforward
```
$rosrun spacenav_node spacenav_node
```
The node is now publishing to the topics listed above.
#### Examples ####
Viewing the output from the spacenav can be done as follows:
```
$ rostopic echo /spacenav/joy
```

