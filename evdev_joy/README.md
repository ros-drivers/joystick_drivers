evdev_joy
============================================================
## 1. Overview
This node uses the linux generic input event interface [evdev](https://en.wikipedia.org/wiki/Evdev) and has therefore the capability to use any `/dev/input/event*` device as an Joystick input.
Moreover it is **force feedback** compatible.

To figure out, what capabilities your Joystick (or other HID) has, the tool `evemu-describe` is very useful.

As you can see, each event device has ***event-types*** and ***event-codes***.

- ***event-types*** are simply like 'Categories' for example:
    - `EV_KEY`for Keys and Buttons
    - `EV_ABS`for absoloute Axes (like in a joystick)
    - `EV_REL`for relative Axes (like in a mouse)
    -   ...
- ***event-codes*** specify the individual button or joystick-thumb or ...
    - the tool `evemu-record` shows the assigned code for each device button or axes.

**(Note, that this Node currently only supports types `EV_KEY` and `EV_ABS`)**


## 2. Installation


## 3. Topics
#### Publishing
- **joy** (http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html): outputs the joystick state.
    - Key and Buttons between *pressed: 1.0* and *released: 0.0*.
    - Axes between *-1.0* and *1.0* (-100%...100%) if the axes has a positive and negative postion. Otherwise between
      *0.0* and *1.0* (0%...100%).
    - To setup the array postion for each axes or button use parameters explained in the following.
#### Subscribing
*(Only, if the Device has a Feedback **EV_FF**)*
- **set_feedback** (http://docs.ros.org/api/sensor_msgs/html/msg/JoyFeedbackArray.html): the force feedback instruction(s).
    - the Array contains *n* feedback-[events](http://docs.ros.org/api/sensor_msgs/html/msg/JoyFeedback.html)
        - *type*: this node only handles event with type = 1 (TYPE_RUMBLE)
        - *id*: this is the effect id, currently the node knows two effects:
            - *RUMBLE_HEAVY* (0): rumbles the strong motor
            - *RUMBLE_LIGHT* (1): rumbles the light motor
        - *intensity*: the intensity between *0.0* and *1.0* (0%...100%)
    - each event runs forever, until you will set its intensity to *0.0*
    - whenever the feedback array in *set_feedback* is published for each Array-entry:
        - the node will start a new event, if this array index has never been used before
        - overwrites existing events or reset the intensity of an event, if this array index has been already used 
        - does nothing with this event, if the feedbacktype is set to 0
    
## 4. Parameters
- ***device_file_path*** the device file. Mostly: `/dev/input/event#`
- ***buttons_mapping*** *(array)* this setups the published buttons in the *joy-Message*. Please use the *event_codes*
  from type `EV_KEY`. 
- ***axes_mapping*** *(array)* this setups the published axes in the *joy-Message*. Please use the *event_codes*
  from type `EV_ABS`. 
- ***max_send_frequency*** this param prevents the node from publishing with a to high frequency. The messages are held
  back to meet this maximum frequency. Notice, that a Button-Event wil be send immediately to not miss events.


#### Example
Have a look at the sample launch-files:
```
<launch>
  <node pkg="evdev_joy" type="evdev_joystick_node" name="joystick" output="screen">
        <param name="device_file_path" value="/dev/input/event23"/>
        <param name="max_send_fequency" value="200"/>
        <rosparam param="buttons_mapping">  [BTN_SOUTH,
                                            BTN_EAST,
                                            BTN_NORTH,
                                            BTN_WEST,
                                            BTN_TL,
                                            BTN_TR,
                                            BTN_SELECT,
                                            BTN_START,
                                            BTN_MODE,
                                            BTN_THUMBL,
                                            BTN_THUMBR]
        </rosparam>
    <rosparam param="axes_mapping">         [ABS_X,
                                            ABS_Y,
                                            ABS_Z,
                                            ABS_RX,
                                            ABS_RY,
                                            ABS_RZ,
                                            ABS_HAT0X,
                                            ABS_HAT0Y]
    </rosparam>
  </node>
</launch>
```