evdev_joy
============================================================
## 1. Overview
This node uses the linux generic input event interface [evdev](https://en.wikipedia.org/wiki/Evdev) and has therefore the capability to use any `/dev/input/event*` device as an Joystick input.
Moreover it is **force feedback** compatible.

## 2. Installation

## 3. Topics
#### Publishing
- **joy** (http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html): outputs the joystick state.
    - Key and Buttons between *pressed: 1.0* and *released: 0.0*.
    - Axes between *-1.0* and *1.0* (-100%...100%) if the axes has a postive and nagative postion. Otherwise between *0.0* and *1.0* (0%...100%). 
#### Subscribing
*(Only, if the Device has a Feedback **EV_FF**)*
- **set_feedback** (http://docs.ros.org/api/sensor_msgs/html/msg/JoyFeedbackArray.html): the force feedback instruction(s).
    - the Array contains *n* feedback-[events](http://docs.ros.org/api/sensor_msgs/html/msg/JoyFeedback.html)
        - *type*: this node only handles event with type = 1 (TYPE_RUMBLE)
        - *id*: this is the effect id, currently the node knows two effects:
            - *RUMBLE_HEAVY* (0): rumbles the strong motor
            - *RUMBLE_LIGHT* (1): rumbles the light motor
        - *intensity*: the intensity between *0.0* and *1.0* (0%...100%)
    - each event runs forewever, until you will set its intensity to *0.0*
    - whenever the feedback array in *set_feedback* is published for each Array-entry:
        - the node will start a new event, if this array index has never been used before
        - overwrites existing events or reset the intensity of an event, if this arrray index has been already used 
        - does nothing with this event, if the feedbacktype is set to 0
    
## 4. Parameters
- 

