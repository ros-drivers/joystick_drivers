#include "evdev.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "evdev_joy");

    ModernJoystick joyer(ros::NodeHandle(""), ros::NodeHandle("~"));
    ros::AsyncSpinner asyncSpinner(1); //A second thread is needed, because this thread will
                                        //Block in joyer.run() 
    asyncSpinner.start();
    while (ros::ok())
    {
        joyer.run();
    }
    asyncSpinner.stop();
}

ModernJoystick::ModernJoystick(ros::NodeHandle nh, ros::NodeHandle pnh) : _nodeHandle(nh),
                                                                          _privateNodeHandle(pnh),
                                                                          _joyDevName("/dev/input/event10"),
                                                                          _maxSendFrequency(100),
                                                                          _buttonsMappingParam{"BTN_SOUTH",
                                                                                               "BTN_EAST",
                                                                                               "BTN_NORTH",
                                                                                               "BTN_WEST",
                                                                                               "BTN_TL",
                                                                                               "BTN_TR",
                                                                                              "BTN_SELECT",
                                                                                              "BTN_START",
                                                                                               "BTN_MODE",
                                                                                               "BTN_THUMBL",
                                                                                               "BTN_THUMBR"},
                                                                          _axesMappingParam{"ABS_X",
                                                                                            "ABS_Y",
                                                                                            "ABS_Z",
                                                                                            "ABS_RX",
                                                                                            "ABS_RY",
                                                                                            "ABS_RZ",
                                                                                            "ABS_HAT0X",
                                                                                            "ABS_HAT0Y"}
{
    this->init();
}

void ModernJoystick::connect(){
    /**
     * Open Device
     */
    do{
    ModernJoystick::_joyFD = open(ModernJoystick::_joyDevName.c_str(), O_RDWR); 
    ROS_ERROR_COND(_joyFD < 0, "Failed to open Device '%s' . %s \n\r Retry in 2 seconds.", _joyDevName.c_str(),strerror(errno)); 
    }while(_joyFD < 0 && ros::Duration(2).sleep());
    int err = libevdev_new_from_fd(_joyFD, &_joyDEV);
    if(err < 0){
        ROS_ERROR( "Filed to init libevdev - Quit Node. %s", strerror(-err));
        ros::shutdown(); //If this Appairs, there must be a Hardwareproblem or incompability.
    }
    ROS_INFO("Connected to %s.", libevdev_get_name(_joyDEV));
}


void ModernJoystick::init()
{
    /**
     * Check Params
     */
    _privateNodeHandle.param<std::string>("device_file_path", _joyDevName, _joyDevName);
    _privateNodeHandle.param<std::vector<std::string>>("buttons_mapping", _buttonsMappingParam, _buttonsMappingParam);
    _privateNodeHandle.param<std::vector<std::string>>("axes_mapping", _axesMappingParam, _axesMappingParam);
    _privateNodeHandle.param<int>("max_send_fequency", _maxSendFrequency, _maxSendFrequency);

    this->connect();
    ROS_INFO(_buttonsMappingParam[0].c_str());

    /**
     * Quering Device Capabilitys 
     */
    if (libevdev_has_event_type(_joyDEV, EV_ABS))
    {
        ROS_INFO("Looks like a Controller, with Axes!");
    }
    if (libevdev_has_event_type(_joyDEV, EV_KEY))
    {
        ROS_INFO("Looks like a Controller, with Buttons!");
    }

    //Checking and Calculating Mapping (Axes & Buttons)
    for (int i = 0; i < _buttonsMappingParam.size(); i++)
    {
        std::string buttonName = _buttonsMappingParam[i];
        int eventCode = libevdev_event_code_from_name(EV_KEY, buttonName.c_str());
        if (eventCode < 0)
        {
            ROS_ERROR_STREAM("There is no Button Event (EV_KEY) '" << buttonName << "' in this Device. Skipping this Button!");
        }
        else
        {
            _buttonsMapping.insert(std::make_pair(eventCode, i));
        }
    }

    for (int i = 0; i < _axesMappingParam.size(); i++)
    {
        std::string axesName = _axesMappingParam[i];
        int eventCode = libevdev_event_code_from_name(EV_ABS, axesName.c_str());
        if (eventCode < 0)
        {
            ROS_ERROR_STREAM("There is no Axes Event (EV_ABS) '" << axesName << "' in this Device. Skipping this Axes!");
        }
        else
        {
            _axesMapping.insert(std::make_pair(eventCode, i));
            //Calculating Value Mapping for Axe
            int min = libevdev_get_abs_minimum(_joyDEV, eventCode);
            int max = libevdev_get_abs_maximum(_joyDEV, eventCode);
            int absMax = fmax(abs(min), abs(max));
            _axesAbsMax.insert(std::make_pair(eventCode, absMax));
        }
    }

    



    /**
     * Initing ROS spefic things
     */
    _joyPublisher = _privateNodeHandle.advertise<sensor_msgs::Joy>("joy", 10);
    ROS_INFO_STREAM("Publishing to: " << _joyPublisher.getTopic());
    if (libevdev_has_event_type(_joyDEV, EV_FF))
    {
        _feedbackSubscriber = _privateNodeHandle.subscribe<sensor_msgs::JoyFeedbackArray>("set_feedback", 1, &ModernJoystick::feedbackCallback, this);
        ROS_INFO_STREAM("Joystick seems to have Force-Feedback, Subscribing to: " << _feedbackSubscriber.getTopic());
    }
    ros::Rate rate(_maxSendFrequency);
    _sendTimer = _privateNodeHandle.createTimer(rate, &ModernJoystick::timerCallback, this);
    ROS_INFO_STREAM("Maximum Send Frequency is set to " <<_maxSendFrequency << "Hz .");


    /**
     * Inting The Arrays
     */
    _joyMessage.axes.resize(_axesMappingParam.size());
    _joyMessage.buttons.resize(_buttonsMappingParam.size());
}


void ModernJoystick::publishJoyMessage(){
     //Send
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "joy_link"; //TODO: parametrice
    _joyMessage.header = header;
    _joyPublisher.publish(_joyMessage);
}

void ModernJoystick::run()
{
    
    struct input_event ev;
    bool nowSend = readJoy(ev);
    updateMessage(ev);
    if(nowSend){
        _sendTimer.stop();//Timer is stopped, because all Events are send.  (Before publish, because of Race Conditions!);
        publishJoyMessage();         
    }
    else{
        _sendTimer.start();//Timer start, because new Event, has to be send.
    }
}


void ModernJoystick::timerCallback(const ros::TimerEvent & event){
    _sendTimer.stop(); //Timer is stopped, because all Events are send.
    publishJoyMessage();
}

void ModernJoystick::feedbackCallback(const sensor_msgs::JoyFeedbackArrayConstPtr &msg)
{	
    //For Each Feedback:
    //(Each Array-Slot is one Feedback-Slot in the Device, each feedback on the
    //slot, will overwrite previous Feedbacks on this fitting Device-Slot.
    //If the Array-Slot is empty or not TYPE_RUMBLE, this event will let untouched on Device.)
    for (int i = 0; i < msg->array.size(); i++)
    {
        sensor_msgs::JoyFeedback feedback = msg->array[i];
        if (feedback.type = feedback.TYPE_RUMBLE)
        { //Other Things arent Handled!
            struct ff_effect effect = ff_effect();
            effect.direction = 0;
            effect.id = -1;
            effect.replay.length = 0; //Infinity -> TODO: Parametrize
            switch (feedback.id)
            {
            case RUMBLE_HEAVY:
            {
                effect.type = FF_RUMBLE;
                effect.u.rumble.strong_magnitude = 0xFFFF * feedback.intensity; //TODO: ????
	    }
            break;
            case RUMBLE_LIGHT:
            {
                effect.type = FF_RUMBLE;
                effect.u.rumble.weak_magnitude = 0xFFFF * feedback.intensity; //TODO: ????
            }
            break;
            default:
            {
                //Nothing more implemented yet
                ROS_WARN("This Effect-ID (Type) isn't implemented yet!");
            }
                return;
            }

        //get that Evet-ID of this array Slot
            if (_feedbackDeviceID.count(i) == 0)
            { //New Feedback!
                addEffect(effect);
                playEffect(effect.id);
                _feedbackDeviceID[i] = effect.id;
	    }
            else
            { //Old Feedback has to bee destroyed
                stopEffect(_feedbackDeviceID[i]);
                //ros::Duration(2).sleep();
                removeEffect(_feedbackDeviceID[i]);
                //ros::Duration(2).sleep();
                addEffect(effect);
                //ros::Duration(2).sleep();
                playEffect(effect.id);
                _feedbackDeviceID[i] = effect.id;
            }
        }
    }
}

void ModernJoystick::addEffect(struct ff_effect & effect)
{
    int res = ioctl(_joyFD, EVIOCSFF, &effect);
    if (res < 0)
    {
        ROS_ERROR("Failed to Upload Effect to Joystick. %s", strerror(res));
    }
}

void ModernJoystick::removeEffect(short effectID){
    int res = ioctl(_joyFD, EVIOCRMFF, effectID);
    if (res < 0)
    {
        ROS_ERROR("Failed to Remove Effect from Joystick. %s", strerror(res));
    }
}

void ModernJoystick::playEffect(short effectID){
        struct input_event play = input_event(); //Zero-Init
        play.type = EV_FF;
        play.code = effectID;
        play.value = 3;

        int res = write(_joyFD, &play, sizeof(play));
        if(res < 0){
            ROS_ERROR("Failed to Play Effect on Joystick!");
        }       
}

void ModernJoystick::stopEffect(short effectID){
    struct input_event stop = input_event(); //Zero-Init
    stop.type = EV_FF;
    stop.code = effectID;
    stop.value = 0;

    int res = write(_joyFD, &stop, sizeof(stop));
    if(res < 0){
            ROS_ERROR("Failed to Stop Effect on Joystick!");
    }

}

ModernJoystick::~ModernJoystick()
{
    libevdev_free(_joyDEV);
    close(_joyFD); //TODO: ERROR
}

/**
 * Returns, wether a send is needed, because its an Event like Button, that you can't wait for
 */
bool ModernJoystick::readJoy(struct input_event &ev)
{
    int rs = libevdev_next_event(_joyDEV, libevdev_read_flag::LIBEVDEV_READ_FLAG_NORMAL | libevdev_read_flag::LIBEVDEV_READ_FLAG_BLOCKING, &ev); //BLOCKING!
    if (rs == libevdev_read_status::LIBEVDEV_READ_STATUS_SUCCESS)
    {

    }
    else if (rs == libevdev_read_status::LIBEVDEV_READ_STATUS_SYNC)
    {
        //Resync in Sync-Mode
        ROS_WARN("Controller out of sync!");
        reSyncJoy();
    }
    else
    {
        if (rs == -EAGAIN)
        {
            //No more events
        }
        else
        {
            ROS_ERROR("Error: %s", strerror(rs));
            ROS_ERROR("Trying to Reconnect Controller.");
            _sendTimer.stop();
            this->connect();
        }
    }

    return (ev.type == EV_KEY);
}

void ModernJoystick::updateMessage(const struct input_event &ev)
{
    if (ev.type == EV_KEY && _buttonsMapping.count(ev.code) == 1)
    {
        _joyMessage.buttons[_buttonsMapping[ev.code]] = ev.value;
    }
    else if (ev.type == EV_ABS && _axesMapping.count(ev.code) == 1)
    {
        _joyMessage.axes[_axesMapping[ev.code]] = mapAxesValue(ev.value, ev.code);
    } //Everything else is not handled by this Node
      //Else means: (1) Another Key, or (2) a not mapped Code
}

void ModernJoystick::reSyncJoy()
{
    struct input_event ev;
    int rc = LIBEVDEV_READ_STATUS_SYNC;
    ROS_WARN("resyncing Controller...");
    while (rc == LIBEVDEV_READ_STATUS_SYNC)
    {
        rc = libevdev_next_event(_joyDEV, LIBEVDEV_READ_FLAG_SYNC, &ev);
        if (rc < 0)
        {
            if (rc != -EAGAIN)
            {
                ROS_ERROR("Error while reading Event from Controller: %s", strerror(-rc));
            }
            return;
        }

        //TODO:Sending Messages!
    }
    updateMessage(ev);
    ROS_WARN("Controller is resynct, maybe you've lost some Messages.");
}

float ModernJoystick::mapAxesValue(int value, int evCode)
{
    int absMax = _axesAbsMax[evCode];
    float percVal = float(value) / float(absMax);
    return percVal;
}
