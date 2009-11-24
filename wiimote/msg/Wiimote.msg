#
#          Wiimote message containing one complete Wiimote state

# Note: For ease of manipulation this message often uses
#       int8[] when a bit vector would work. One might
#       wish to use uint8[], but then Python takes
#       the respective structure as a string and disallows
#       item assignment.

int8    INVALID       = -1
float32 INVALID_FLOAT = -1.0

#-----------------------------
# Header
#----------------------

Header header

#----------------------
# Gyro reading
#-----------------------------
# In radians/sec. If reading is invalid,
# for instance b/c no gyro is attached to the Wii, then
# set first element of covariance to -1 (same as imu_data.msg):
# Covariance matrix (roll, pitch, yaw) in radians^2/sec^2.

geometry_msgs/Vector3 angular_velocity_zeroed
geometry_msgs/Vector3 angular_velocity_raw
float64[9] angular_velocity_covariance

#----------------------
# Accelerometer reading
#-----------------------------
# Acceleration in m/sec^2. Covariance matrix
# (x,y,z) in m^2/sec^4.  (all same as imu_data.msg)

geometry_msgs/Vector3 linear_acceleration_zeroed
geometry_msgs/Vector3 linear_acceleration_raw
float64[9] linear_acceleration_covariance

#----------------------
# Wiimote buttons
#-----------------------------
# Mappings from Wiimote button name
# to array position are defined in wiimoteConstants.py.
#
bool[11] buttons

#----------------------
# Wiimote LED states:
#-----------------------------

bool[4] LEDs

#----------------------
# Wiimote Rumble
#-----------------------------
# State (True or False)

bool rumble

#----------------------
# IR Light sensor (Camera)
#-----------------------------
# The Wiimote handles up to four light sources, 
# and the wiimote_node.py software is written to 
# that limit as well. For future expansion
# we make the following array extensible, rather
# than locking its length down to four:

wiimote/IR_source_info[] ir_tracking

#----------------------
# Wiimote battery
#-----------------------------
# A battery reading consists of two numbers: 
# the battery percentage, and the raw reading.
# Maximum battery is 208 units (unknown how this
# relates to electrical properties): 

float32 raw_battery
float32 percent_battery

#----------------------
# Time of most recent zeroing:
#-----------------------------

roslib/Time zeroing_time

#----------------------
# Error vector
#-----------------------------
# For error condition definitions see wiimoteConstants.py
# Value of zero means all is well. (Currently NOT used).

uint64 errors