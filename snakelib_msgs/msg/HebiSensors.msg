# This is a message that holds the sensory readings data from the HEBI modules. 
#
# Each of the HEBI module has the following sensors:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s), 
#  * the effort that is applied in the joint (Nm or N),
#  * the linear accelerations (m/s^2),
#  * the angular velocities (rad/s), and
#  * the orientations (quaternion representation).
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct states.
# 
# lin_acc, ang_vel contain 3 lists corresponding to the x, y, and z components.
# orientation contains 4 lists corresponding to the x, y, z, and w values.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort
Vec3List lin_acc
Vec3List ang_vel
QuatList orientation
float64[] effort_limit_state
float64[] position_limit_state
float64[] velocity_limit_state
float64[] temperature_state
float64[] safety_led
float64[] voltage
float64[] motor_winding_current
