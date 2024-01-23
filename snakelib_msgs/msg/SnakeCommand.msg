# Defines a "snake command", which defines some behavior we want the snake to carry out. 
# Examples include gaits, controllers, and head-look.
# Name of the command, e.g. "sidewinding" 
string command_name 
# Name of the particular command parameter, e.g. "spatial_frequency"
string[] param_name
# Value for the command parameter
float64[] param_value