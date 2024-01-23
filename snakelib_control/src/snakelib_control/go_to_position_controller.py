import numpy as np
import rospy
from sensor_msgs.msg import JointState

from snakelib_control.abstract_controller import AbstractController

class GoToPositionController(AbstractController):
    """Defines a simple controller that takes the current joint angles and goes to that position."""

    def __init__(self, snake_type, go_to_joint_angles, current_joint_state):
        """Initialization for GoToPositionController

        Args:
            snake_type: string denoting type of snake
            go_to_joint_angles: array defining joint angles to go_to
            current_joint_state: JointState message of current snake joint states

        Raises:
            ValueError: Invalid snake_type for this controller.
        """
        super().__init__(snake_type)

        # If new go_to_joint angles are all identical to the
        self._go_to_joint_angles = go_to_joint_angles

        # Joint angles we start with at the beginning of the transition into the new gait
        self._start_joint_angles = current_joint_state.position

        # How far into the transition we've progressed, where 0 is no progress, 1 is transition complete
        self._transition_progress = 0.0

        self._last_time = (
            current_joint_state.header.stamp
        )  # used to keep track of actual elapsed time

        # Total "snake time" in seconds to take when transitioning between gaits
        self._transition_time = self._snake_param.get("go_to_position_controller", {}).get(
            "transition_time", 1.5
        )

    def set_go_to_joint_angles(self, go_to_joint_angles, current_joint_state):
        """Set the joint state that the robot will go to.

        Args:
            go_to_joint_angles: array for the desired joint state to go_to. Only positions are used.
            current_joint_state: JointState message of current snake joint states
        """
        if not np.array_equal(go_to_joint_angles, self._go_to_joint_angles):
            self._go_to_joint_angles = go_to_joint_angles
            self._start_joint_angles = current_joint_state.position
            self._transition_progress = 0.0

    def controller_name(self):
        """Overrides AbstractController.controller_name."""
        return "go_to_position_controller"

    def update(self, current_joint_state, exiting_head_look=False, n_exiting_modules=2):
        """Overrides AbstractController.update. Simply returns desired go_to joint state."""

        # Get the time from the joint state header and update snake time
        current_time = current_joint_state.header.stamp
        dt = current_time - self._last_time
        self._last_time = current_time
        num_modules = len(current_joint_state.name)
        transition_mask = exiting_head_look * np.array([0]*n_exiting_modules + [1.0] * (num_modules-n_exiting_modules))
        transition_progress = np.maximum(self._transition_progress, transition_mask)
        # Interpolate between joint angles
        desired_joint_angles = np.add(
            self._start_joint_angles,
            np.subtract(
                self._go_to_joint_angles,
                self._start_joint_angles,
            )
            * transition_progress,
        )

        # Update transition progress
        self._transition_progress += np.abs(dt.to_sec()) / self._transition_time
        self._transition_progress = min(self._transition_progress, 1.0)

        desired_joint_state = JointState()
        desired_joint_state.name = self._module_names
        desired_joint_state.position = desired_joint_angles

        # HEBI API requires NaNs to ignore velocity and effort values
        desired_joint_state.velocity = np.array(
            [np.nan] * len(self._go_to_joint_angles)
        )
        desired_joint_state.effort = np.array(
            [np.nan] * len(self._go_to_joint_angles)
        )

        return desired_joint_state
