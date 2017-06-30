# Wiimote Nodes

Additional Documentation
 * [Testing procedure](doc/testing.md)
 * [ROS Wiki](http://wiki.ros.org/wiimote)
 
Tutorials
 * [Teleop with Wiimote](doc/tutorials/teleop.md)

## wiimote_node.py

Original Python version of the wiimote node.

## wiimote_node

The C++ implementation was designed with focus on reduced resource consumption.

### Parameters

* `bluetooth_addr` [str] - Bluetooth address for pairing. Default: `00:00:00:00:00:00` (first device)
* `pair_timeout` [int] - Pair timeout in seconds. `-1` means never timeout. Default: `5`

### Differences from Python Implementation
* Both "/wiimote/nunchuk" and "/wiimote/classic" topics are only published
if the Nunchuk or Classic Controller are connected to the wiimote respectively.
* Wiimote data is only polled from the controller if the data is required
to publish for a topic which has at least one subscriber.
* During Calibration the joysticks (Nunchuk & Classic Controller) are polled
to find the current position which is assumed to be the center. Not all joysticks
rest at the theoretical center position.
* Joysticks (Nunchuk & Classic Controller) minimum and maximum values start at
80% of the theoretical minimum/maximum values and are dynamically updated as
the joystick is used. The full range of values may not be reached on all joysticks
due to physical limitations. Without this adjust, the full range from -1.0 to 1.0
would not be reached on many joysticks.
* Joysticks (Nunchuk & Classic Controller) center deadzone is scaled based on
the granularity of the joystick; python code assumed the same for all.
* Connecting an accessory (Nunchuk, Classic Controller, or MotionPlus) will
automatically start a new calibration sequence.
