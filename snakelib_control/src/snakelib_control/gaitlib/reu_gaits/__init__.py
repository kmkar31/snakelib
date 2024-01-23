import os
import numpy as np
from snakelib_control.gaitlib.gaitlib import Gaitlib

"""
Provides gaits for the ReUnified (ReU) snake. It is expected that SEA snake and R snake
will reuse most (if not all) of these implementations.
"""


class ReuGaits(Gaitlib):
    def create_gait(self):
        pass
    
    snake_type = "REU"
    num_modules = 16  # number of modules
    num_gait_param = 10  # number of gait parameters

    @property
    def gait_params_filepath(self):
        dir_name = os.path.dirname(__file__)
        return os.path.join(dir_name, "reu_gait_parameters.yaml")

    @staticmethod
    def flip_axes(target_angles):
        """
        The direction of the target angles has to flip signs after every pair of actuators,
        since on ReU the rotation axes of subsequent even or odd joints is flipped as
        the robot is assembled.
        """
        direction = 1
        for i in range(len(target_angles[0])):
            target_angles[:, i] *= direction
            if not ((i + 1) % 2):  # every other element, change the direction
                direction *= -1

        return target_angles

    # Import methods defined in other files
    from .compound_serpenoid import compound_serpenoid
    from .head_look import head_look
    from .head_look_ik import head_look_ik
    from .lateral_undulation import lateral_undulation
    from .linear_progression import linear_progression
    from .rolling import rolling
    from .rolling_helix import rolling_helix
    from .rolling_in_shape import rolling_in_shape
    from .slithering import slithering
    from .turn_in_place import turn_in_place
    from .conical_sidewinding import conical_sidewinding

