#!/usr/bin/env python3

import hebi
import rospy
import numpy as np
from functools import partial
from sensor_msgs.msg import JointState
from snakelib_hebi.hebi_interface import HEBIROSWrapper


class HEBIControlROSWrapper(HEBIROSWrapper):
    r"""Driver node that commands targets to the HEBI modules from the `joint_command` topic."""

    def __init__(self):
        # Establish communication with the HEBI modules.
        super().__init__()

        # Start joint command subscriber.
        self.joint_command_topic = "/snake/joint_commands"
        rospy.Subscriber(
            self.joint_command_topic,
            JointState,
            self.joint_command_callback,
            queue_size=None,
        )

        # Initialize command message buffer.
        self.robot_cmd = hebi.GroupCommand(self.num_modules)

        # Send commands to the hebi modules on receiving a new command.
        rospy.spin()

    def send_joint_commands(self):
        r"""Sends joint commands to the module."""
        if (
            np.isnan(self.robot_cmd.position).any()
            and np.isnan(self.robot_cmd.velocity).any()
            and np.isnan(self.robot_cmd.effort).any()
        ):
            rospy.logwarn_throttle(100, f"No joint command sent.")
            return

        self.robot.send_command(self.robot_cmd)

    def joint_command_callback(self, msg):
        r"""Processes and sends joint commands to the HEBI modules.

        Populates the robot_cmd object with appropriate fields from the received msg after ordering and filtering
        received messages.

        Args:
            msg (JointState): JointState message containing the joint commands.

        """
        self.robot_cmd.position, self.robot_cmd.velocity, self.robot_cmd.effort = map(
            partial(self.sort_joint_commands, names_list=msg.name),
            [msg.position, msg.velocity, msg.effort],
        )

        self.send_joint_commands()

    def sort_joint_commands(self, unsorted_cmds, names_list=None):
        r"""Rearranges the joint commands to match order of the module names as provided in `HEBIROSWrapper`'s
        instantiation.

        Rearranging is done by matching the provided `names_list` to the saved `module_names` attribute. If not provided,
        no commands will be executed and warning will be issued. NaNs are returned if `unsorted_cmds` does not match
        the number of modules.

        Args:
            unsorted_cmds (list): List of joint commands.
            names_list (list): List of module names.

        Returns:
            sorted_cmds (list): List of joint commands sorted by module names.

        """
        if len(names_list) != self.num_modules:
            rospy.logwarn_throttle(
                5,
                f"Message from topic {self.joint_command_topic} either does not contain a joint names list or does not \
                    have correct number of entries. Not executing published commands.",
            )
            return self.num_modules * [
                np.nan
            ]  # HEBI controller requires sending NaNs for no actions.
        elif len(unsorted_cmds) != self.num_modules:
            return self.num_modules * [
                np.nan
            ]  # HEBI controller requires sending NaNs for no actions.
        else:
            joint_commands_with_names = dict(
                zip(names_list, unsorted_cmds)
            )  # Pair module names with commands.
            # Rearrange commands to match saved module names.
            sorted_cmds = [
                joint_commands_with_names.get(module_name)
                for module_name in self.module_names
            ]
        return sorted_cmds
