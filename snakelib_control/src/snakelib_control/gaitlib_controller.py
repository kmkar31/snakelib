import numpy as np
import rospy
from sensor_msgs.msg import JointState

from snakelib_control.abstract_controller import AbstractController
from snakelib_control.gaitlib.reu_gaits import ReuGaits
from snakelib_control.gaitlib.rsnake_gaits import RsnakeGaits
from snakelib_control.gaitlib.sea_gaits import SeaGaits
from .utils import Robot
import time


class GaitlibController(AbstractController):
    """Defines a controller that runs standard open-loop serpenoid gaits

    This class also provides transitions between gaits.

    """

    def __init__(self, snake_type, current_joint_state, snake_time):
        """Initialization for GaitlibController.

        Since this class provides smooth transitions into a desired gait, it must keep
        track of the joint angles at the instance in which the class was either created
        or when a new desired gait is set.

        Args:
            snake_type: string denoting type of snake
            current_joint_state: JointState message of current snake joint states

        Raises:
            ValueError: Invalid snake_type for this controller.
        """
        super().__init__(snake_type)

        # Gaitlib is only valid for these snake types
        if self._snake_type == "REU":
            self._gaitlib = ReuGaits()
        elif self._snake_type == "SEA":
            self._gaitlib = SeaGaits()
        elif self._snake_type == "RSNAKE":
            self._gaitlib = RsnakeGaits()
            
        else:
            raise ValueError("Invalid snake_type.")

        self._gaitlib.num_modules = len(self._module_names)

        # Dictionary defining the current gait parameters.
        self._gait_param_dict = self._snake_param.get("gait_params", {})

        # Joint angles we start with at the beginning of the transition into the new gait
        self._start_joint_angles = current_joint_state.position

        self._desired_gait = ''
        self._desired_gait_param = self._gait_param_dict.get(self._desired_gait, {})

        self.robot = Robot(robot_name=self._snake_type)

        self._last_time = (
            current_joint_state.header.stamp
        )  # used to keep track of actual elapsed time

        self._current_joint_state = current_joint_state

        self._snake_time = snake_time  # snake time used as input to the gait equations.

        # Total "snake time" in seconds to take when transitioning between gaits
        self._transition_time = self._snake_param.get("gaitlib_controller", {}).get(
            "transition_time", 1.5
        )

        # How far into the transition we've progressed, where 0 is no progress, 1 is transition complete
        self._transition_progress = 0.0

    @property
    def snake_time(self):
        return self._snake_time

    @property
    def gait_name(self):
        return self._desired_gait

    def controller_name(self):
        """Overrides AbstractController.controller_name."""
        return "gaitlib_controller"

    def update_gait(self, snake_command, start_joint_state, transition_override=False):
        """Updates the desired gait and gait parameters from a SnakeCommand message.

        The current joint angles are required so that the snake can smoothly transition
        into the new desired gait.

        Args:
            snake_command (SnakeCommand): Message defining gait name and gait parameters.
            start_joint_state (JointState): Start joint angle for the interpolation.
            transition_override (bool): Flag to cancel transition interpolation.
        """
        if self.is_gaitlib_command(snake_command):
            self._current_joint_state = start_joint_state
            # Reset transition progress only if gait changes.
            if not (snake_command.command_name == self._desired_gait):
                print("Gait Transition")
                self._start_joint_angles = start_joint_state.position
                self._transition_progress = 0.0
            self._desired_gait = snake_command.command_name

            """Update gait parameters dictionary. First get the dictionary for the current
            desired gait, then update the parameters one by one in that dictionary.
            """
            for param_name, param_value in zip(snake_command.param_name, snake_command.param_value):
                gait_dict = self._gait_param_dict.get(self._desired_gait, {})
                gait_dict.update({param_name: param_value})
                self._gait_param_dict[self._desired_gait] = gait_dict

            # Get the desired gait parameters from the parameters dictionary
            self._desired_gait_param = self._gait_param_dict.get(self._desired_gait, {})

        if transition_override:
            self._transition_progress = 1.0

    @staticmethod
    def is_gaitlib_command(command):
        """Checks if a given SnakeCommand is a valid command to use with Gaitlib.

        Args:
            command: SnakeCommand that will be checked

        Returns:
            True if SnakeCommand is a valid command for Gaitlib, False otherwise.
        """

        if command.command_name in [
            "lateral_undulation",
            "linear_progression",
            "rolling_helix",
            "rolling",
            "rolling_in_shape",
            "conical_sidewinding",
            "slithering",
            "turn_in_place",
            "head_look",
            "head_look_ik",
        ]:
            return True
        else:
            return False

    def get_joint_angles(self, gait_name, snake_time, gait_param):
        """Provides the joint angles from gaitlib.

        Args:
            gait_name: string denoting name of gait
            snake_time: double denoting the time for the gait equation
            gait_param: dictionary of gait parameters

        Returns:
            Array of joint angles

        Raises:
            ValueError: Invalid snake_type for this controller.
        """
        controller_kwargs = self._prepare_inputs(gait_name, snake_time, gait_param)
        joint_angles = getattr(self._gaitlib, gait_name)(**controller_kwargs)

        return joint_angles

    def _prepare_inputs(self, gait_name, snake_time, gait_param):
        """Prepares inputs for gaitlib controllers.

        Args:
            gait_name: string denoting name of gait
            snake_time: double denoting the time for the gait equation
            gait_param: dictionary of gait parameters

        Returns:
            controller_kwargs: Dictionary of inputs for gaitlib controllers
        """
        controller_kwargs = dict(
            t=snake_time,
            params=gait_param,
        )

        if gait_name in ['rolling_in_shape']:
            controller_kwargs.update(dict(current_angles=self._start_joint_angles))
        if gait_name in ['head_look_ik']:
            controller_kwargs.update(dict(current_angles=self._current_joint_state.position, robot=self.robot))
        if gait_name in ['head_look']:
            controller_kwargs.update(dict(current_angles=self._current_joint_state.position))
        elif gait_name == 'rolling_helix':
            controller_kwargs.update(dict(pole_params=self._gait_param_dict.get('pole_climb', {})))

        return controller_kwargs

    def update(self, current_joint_state):
        """Overrides AbstractController.update.

        Args:
            current_joint_state: JointState message for current snake joint state
        """
        # Pack JointState message
        joint_state_msg = JointState()
        joint_state_msg.name = self._module_names
        joint_state_msg.velocity = np.array(
            [np.nan] * len(joint_state_msg.position)
        )
        joint_state_msg.effort = np.array(
            [np.nan] * len(joint_state_msg.position)
        )

        # Race condition handing.
        # If received snake command is changed between instantiating a gaitlib controller
        # and updating gait params, race condition occurs. E.g., if hold_position is received,
        # code will break as gaitlib controller doesn't recognize hold position.
        # Return start joint angles in this case.
        # Ideally, we want to return previous joint command, but start joint angles is fine
        # since this race will only occur between instiantiating a new controller and updating
        # its params.
        if self._desired_gait == '':
            joint_state_msg.position = self._start_joint_angles
            return joint_state_msg

        # Get the time from the joint state header and update snake time
        wave_direction = self._desired_gait_param.get('speed_multiplier', 1)
        pole_direction = np.sign(self._desired_gait_param.get('pole_direction', 1))

        current_time = current_joint_state.header.stamp

        # Potentially refactor headlook as a seperate class of controller.
        headlook_multiplier = 0 if self._desired_gait == 'head_look' else 1

        transition_dt = current_time - self._last_time
        snake_dt = (headlook_multiplier)*(pole_direction)*(wave_direction)*transition_dt
        self._snake_time = self._snake_time + snake_dt.to_sec()
        self._last_time = current_time

        # Determine joint angles. Note that gaits for REU, SEA, and RSNAKE are identical
        desired_gait_joint_angles = self.get_joint_angles(
            self._desired_gait, self._snake_time, self._desired_gait_param
        )

        # Interpolate between joint angles
        joint_angles = np.add(
            self._start_joint_angles,
            np.subtract(
                desired_gait_joint_angles,
                self._start_joint_angles,
            )
            * self._transition_progress,
        )

        if self._desired_gait == 'head_look':
            joint_angles = desired_gait_joint_angles

        # Update transition progress
        self._transition_progress += np.abs(transition_dt.to_sec()) / self._transition_time
        self._transition_progress = min(self._transition_progress, 1.0)

        joint_state_msg.position = joint_angles
        return joint_state_msg
