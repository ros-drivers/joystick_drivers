# Wiimote message containing one complete Wiimote state

Header header

# Gyro reading:

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

# Accelerometer reading:

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 

# Wiimote buttons. Mappings from Wiimote button name

# to array position are defined in wiimoteConstants.py
int32[] buttons

# Wiimote LED states:
int32[4] LEDs

# Wiimote Rumble state (1 or 0):
int32 rumble

# Error vector. For error condition definitions see wiimoteConstants.py
# Value of zero means all is well.
uint64 errors