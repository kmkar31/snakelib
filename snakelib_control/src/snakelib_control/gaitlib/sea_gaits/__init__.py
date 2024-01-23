from snakelib_control.gaitlib.reu_gaits import ReuGaits

"""
Defines gaites for the SEA snake, which are identical to the ReU snake gaits.
"""


class SeaGaits(ReuGaits):

    snake_type = "SEA"
    num_modules = 16
    num_gait_param = 10
