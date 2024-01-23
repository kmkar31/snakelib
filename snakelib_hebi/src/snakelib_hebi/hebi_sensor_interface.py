#!/usr/bin/env python3

import hebi
import rospy
from snakelib_msgs.msg import HebiSensors
from snakelib_hebi.hebi_interface import HEBIROSWrapper
from sensor_msgs.msg import JointState


class HEBISensingROSWrapper(HEBIROSWrapper):
    """Driver node that publishes sensor (joint angle, joint velocity, effort, linear acceleration, angular velocity, and orientation) data from the HEBI modules."""

    def __init__(self):
        # Establish communication with the HEBI modules.
        super().__init__()

        # Initialize required publishers.
        self.hebi_sensors_pub = rospy.Publisher(
            "/hebi_sensors/data", HebiSensors, queue_size=10
        )
        self.joint_state_pub = rospy.Publisher(
            "/snake/joint_states", JointState, queue_size=10
        )

        # Initialize feedback message buffers.
        self.hebi_sensors = HebiSensors()
        self.joint_state = JointState()

        # Set loop rate as feedback frequency.
        self.loop_rate = self.feedback_frequency
        self.robot_feedback = hebi.GroupFeedback(self.num_modules)

    def run_loop(self):
        r"""Send commands regularly until shutdown."""
        self.update_feedback()
        self.hebi_sensors_pub.publish(self.hebi_sensors)
        self.joint_state_pub.publish(self.joint_state)

    def update_feedback(self):
        r"""Populates respective attributes with updated sensor readings."""
        # Read raw feedback from the module sensors.
        feedback = self.get_feedback()

        # Populate hebi sensors message with feedback.
        self.hebi_sensors.header.stamp = rospy.Time.now()
        self.hebi_sensors.name = self.module_names
        self.hebi_sensors.position = feedback["position"]
        self.hebi_sensors.velocity = feedback["velocity"]
        self.hebi_sensors.effort = feedback["effort"]

        self.hebi_sensors.lin_acc.x = feedback["lin_acc"][:, 0]
        self.hebi_sensors.lin_acc.y = feedback["lin_acc"][:, 1]
        self.hebi_sensors.lin_acc.z = feedback["lin_acc"][:, 2]

        self.hebi_sensors.ang_vel.x = feedback["ang_vel"][:, 0]
        self.hebi_sensors.ang_vel.y = feedback["ang_vel"][:, 1]
        self.hebi_sensors.ang_vel.z = feedback["ang_vel"][:, 2]

        self.hebi_sensors.orientation.x = feedback["orientation"][:, 0]
        self.hebi_sensors.orientation.y = feedback["orientation"][:, 1]
        self.hebi_sensors.orientation.z = feedback["orientation"][:, 2]
        self.hebi_sensors.orientation.w = feedback["orientation"][:, 3]
        
        self.hebi_sensors.effort_limit_state = feedback["effort_limit_state"]
        self.hebi_sensors.position_limit_state = feedback["position_limit_state"]
        self.hebi_sensors.velocity_limit_state = feedback["velocity_limit_state"]
        self.hebi_sensors.temperature_state = feedback["temperature_state"]
        self.hebi_sensors.safety_led = feedback["safety_led"]
        self.hebi_sensors.voltage = feedback["voltage"]
        self.hebi_sensors.motor_winding_current = feedback["motor_winding_current"]

        self.joint_state.header = self.hebi_sensors.header
        self.joint_state.name = self.module_names
        self.joint_state.position = self.hebi_sensors.position
        self.joint_state.velocity = self.hebi_sensors.velocity
        self.joint_state.effort = self.hebi_sensors.effort

        # Perform filtering on the raw data.
        # TODO: In case filtering is required, add here.
        '''
        for i in range(len(self.hebi_sensors.effort_limit_state)):
            if self.hebi_sensors.safety_led[i] != 0:
                print("Module", self.hebi_sensors.name[i], "LED Orange")
                print(self.hebi_sensors.position[i], self.hebi_sensors.velocity[i], self.hebi_sensors.effort[i])
                print(self.hebi_sensors.position_limit_state[i], self.hebi_sensors.velocity_limit_state[i], self.hebi_sensors.effort_limit_state[i], self.hebi_sensors.temperature_state[i])
                if self.hebi_sensors.effort_limit_state[i] != 2:
                    print(self.hebi_sensors.name[i], " Torque Safety Controller Active")
                if self.hebi_sensors.position_limit_state[i] != 2:
                    print(self.hebi_sensors.name[i], " Position Safety Controller Active")
                if self.hebi_sensors.velocity_limit_state[i] != 2:
                    print(self.hebi_sensors.name[i], " Velocity Safety Controller Active")
        '''

    def get_feedback(self):
        r"""Returns raw feedback from the module sensors. Uses old feedback, should new feedback be not available.

        Returns:
            feedback (dict): Dictionary containing the feedback from the module sensors.
                - joint positions (list): List of joint positions.
                - joint velocities (list): List of joint velocities.
                - joint efforts (list): List of joint efforts.
                - linear accelerations (list): List of 3D linear accelerations.
                - angular velocities (list): List of 3D angular velocities.
                - orientation (list): List of quaternion orientation.
        """
        fbk_old = self.robot_feedback
        self.robot_feedback = self.robot.get_next_feedback(
            reuse_fbk=self.robot_feedback
        )

        if self.robot_feedback is None:
            self.robot_feedback = fbk_old

        feedback = {
            "position": self.robot_feedback.position,
            "velocity": self.robot_feedback.velocity,
            "effort": self.robot_feedback.effort,
            "lin_acc": self.robot_feedback.accelerometer,
            "ang_vel": self.robot_feedback.gyro,
            "orientation": self.robot_feedback.orientation,
            "effort_limit_state" : self.robot_feedback.effort_limit_state,
            "position_limit_state":self.robot_feedback.position_limit_state,
            "velocity_limit_state":self.robot_feedback.velocity_limit_state,
            "temperature_state":self.robot_feedback.temperature_state,
            "safety_led": [x.r for x in self.robot_feedback.led.color],
            "voltage" : self.robot_feedback.voltage,
            "motor_winding_current" : self.robot_feedback.motor_winding_current
        }

        return feedback