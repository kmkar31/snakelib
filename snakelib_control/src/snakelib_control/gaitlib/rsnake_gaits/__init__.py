import os
from snakelib_control.gaitlib.reu_gaits import ReuGaits

"""
Defines gaits for the Rsnake. All R snake gaits are identical to ReU snake gaits.
"""


class RsnakeGaits(ReuGaits):

    snake_type = "RSNAKE"
    num_modules = 14
    num_gait_param = 10

    @property
    def gait_params_filepath(self):
        dir_name = os.path.dirname(__file__)
        return os.path.join(dir_name, "rsnake_gait_parameters.yaml")
