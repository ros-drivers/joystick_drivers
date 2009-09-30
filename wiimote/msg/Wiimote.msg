#
#          Wiimote message containing one complete Wiimote state

# Note: For ease of manipulation this message often uses
#       int8[] when a bit vector would work. One might
#       wish to use uint8[], but then Python takes
#       the respective structure as a string and disallows
#       item assignment.

#-----------------------------
# Header
#----------------------

Header header

#----------------------
# Gyro reading
#-----------------------------
# In radians/sec. If reading is invalid
# for instance b/c no gyro is attached to the Wii, then
# set first element of covariance to -1 (same as imu_data.msg):

geometry_msgs/Vector3 angular_velocity_zeroed
geometry_msgs/Vector3 angular_velocity_raw
float64[9] angular_velocity_covariance

#----------------------
# Accelerometer reading
#-----------------------------
#   (same as imu_data.msg):

geometry_msgs/Vector3 linear_acceleration_zeroed
geometry_msgs/Vector3 linear_acceleration_raw
float64[9] linear_acceleration_covariance

#----------------------
# Wiimote buttons
#-----------------------------
# Mappings from Wiimote button name
# to array position are defined in wiimoteConstants.py.
#
int8[11] buttons

#----------------------
# Wiimote LED states:
#-----------------------------

int8[4] LEDs

#----------------------
# Wiimote Rumble
#-----------------------------
# State (1 or 0)

uint8 rumble

#----------------------
# IR Light sensors
#-----------------------------
# Each light has a 2D position
# If the dimensions are set to -1, then no light was 
# detected for this light

geometry_msgs/Point[4] ir_positions
int64[4] ir_sizes

#----------------------
# Wiimote battery
#-----------------------------
# A battery reading consists of two numbers: 
# the battery percentage, and the raw reading: 

float32[2] battery

#----------------------
# Time of most recent zeroing:
#-----------------------------

float64 zeroing_time

#----------------------
# Error vector
#-----------------------------
# For error condition definitions see wiimoteConstants.py
# Value of zero means all is well.

uint64 errors