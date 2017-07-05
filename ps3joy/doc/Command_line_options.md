### Command-Line Options ### 
In an attempt to run either ps3joy.py or ps3joy_node.py, paramaters can be added to enable features.
   
```
rosrun ps3joy ps3joy.py --help
usage: ps3joy.py [--inactivity-timeout=<n>] [--no-disable-bluetoothd] [--redirect-output] [--continuous-output]=<f>
<n>: inactivity timeout in seconds (saves battery life).
<f>: file name to redirect output to.
``` 

`--inactivity-timeout` 
  This may be useful for saving battery life and reducing contention on the 2.4 GHz network.Your PS3 controller 
  will shutdown after a given amount of time of inactivity.  

`--no-disable-bluetoothd` 
   ps3joy.py will not take down bluetoothd. Bluetoothd must be configured to not handle input device, otherwise
   you will receive an error saying "Error binding to socket". 

`--redirect-output`
   This can be helpful when ps3joy.py is running in the backgound. This will allow the standard output
   and error messages to redirected into a file.   

`--continuous-output`
   This will output continuous motion streams and as a result this will no longer leave extended periods of 
   no messages on the /joy topic. ( This only works for ps3joy.py. Entering this parameter in ps3joy_node.py will
   result in the parameter being ignored.)

