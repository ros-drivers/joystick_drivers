## Packages that need to be installed
----------------------------------
* joystick
* libusb-dev
* bluez-5.37
* Pairing instructions
--------------------

1. If you can connect the joystick and the bluetooth dongle into the same 
   computer connect the joystick to the computer using a USB cable.

2. Load the bluetooth dongle's MAC address into the ps3 joystick using:
```
sudo bash
rosrun ps3joy sixpair
```
  If you cannot connect the joystick to the same computer as the dongle,
  find out the bluetooth dongle's MAC address by running (on the computer
  that has the bluetooth dongle):
```
hciconfig
```
  If this does not work, you may need to do
```
sudo hciconfig hci0 up
```
  and retry
```
hciconfig
```
3. Plug the PS3 joystick into some other computer using a USB cable.
   
4. Replace the joystick's mac address in the following command: 
```
sudo rosrun ps3joy sixpair 01:23:45:67:89:ab
```
## Starting the PS3 joystick
-------------------------
5. Run the following command
```
rosrun ps3joy ps3joy.py
```
6. Open a new terminal and reboot bluez and run joy with: 
```
sudo systemctl restart bluetooth 
rosrun joy joy_node  
```
7. Open a new terminal and echo the joy topic 
```
rostopic echo joy
```
8. This should make a joystick appear at /dev/input/js?

9. You can check that it is working with
  jstest /dev/input/js?
  (replace ? with the name of your joystick)

