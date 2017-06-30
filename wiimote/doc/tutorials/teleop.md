# Wiimote Teleop Sample Node

This sample ROS Node subscribes to the wiimote topics

    /wiimote/nunchuk
    /wiimote/state

And publishes command velocity used for motion

    /cmd_vel

Either the four way D-pad of the Wiimote or the Joystick of the Nunchuk
can be used for controlling the values published for `/cmd_vel`.
**Note:** The Joystick and D-pad are mutually exclusive; if one is in
use the other is ignored.

## ROS parameters
    base (string)
Base name space string to prepend to velocity ROS parameters.

    linear/x/max_velocity (double)
Maximum linear velocity (in m/s)

    linear/x/min_velocity (double)
Minimum linear velocity (in m/s). Setting this to 0.0 will disable backwards
motion. When unspecified, -max_velocity is used.

    angular/z/max_velocity (double)
Maximum angular velocity (in rad/s)

    angular/z/min_velocity (double)
Minimum angular velocity (in rad/s). Setting this to 0.0 will disable
counter-clockwise rotation. When unspecified, -max_velocity is used.

    linear/x/throttle_percent (double)
Default linear throttle percentage (0.0 to 1.0). Adjustable via the
'1' and '+/-' buttons on the Wiimote. Defaults to 0.75.
Private parameter which does not use the 'base'.

    angular/z/throttle_percent (double)
Default angular throttle percentage (0.0 to 1.0). Adjustable via the
'2' and '+/-' buttons on the Wiimote. Defaults to 0.75.
Private parameter which does not use the 'base'.


## Button Definitions
| Button         | Function                           |
|:--------------:| ---------------------------------- |
| D-Pad Up       | Move Forward                       |
| D-Pad Down     | Move Reverse                       |
| D-Pad Left     | Turn/Spin Left                     |
| D-Pad Right    | Turn/Spin Right                    |
| B (Trigger)    | 2x Speed (up to Max)               |
| A              | 1/4 Speed                          |
|                |                                    |
| Home           | Display Wiimote battery level      |
|                |                                    |
| 1\*            | Linear Throttle (defaults to 75%)  |
| 2\*            | Angular Throttle (defaults to 75%) |
| +              | Increase Throttle (rumbles at max) |
| -              | Decrease Throttle (rumbles at min) |
|                |                                    |
| Joystick Up    | Move Forward                       |
| Joystick Down  | Move Reverse                       |
| Joystick Left  | Turn/Spin Left                     |
| Joystick Right | Turn/Spin Right                    |
| Z (Trigger)    | 2x Speed (up to Max)               |
| C (Trigger)    | 1/4 Speed                          |
|                |                                    |

\* Throttle level displayed on Wiimote LEDs

### Launch File Examples
Launch files are provided with usage examples using the turtlebot
simulator for both Python (turtlesim_py.launch) and C++ (turtlesim_cpp.launch).

Requires the installation of ros-<release>-turtlesim ROS package.
