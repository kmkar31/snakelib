#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import (Point, Pose, Quaternion, TransformStamped,
                               Vector3)
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from snakelib_control.utils import JOINT_NAMES, Robot
from snakelib_msgs.msg import HebiSensors
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import StaticTransformBroadcaster


class SnakeStateVisualizer:
    """Publishes data for visualizing snake state in RViz."""

    def __init__(self):
        # Initialize required publishers.
        self.joint_state_pub = rospy.Publisher(
            "/visualization/joint_states", JointState, queue_size=10
        )

        rospy.Subscriber(
            '/hebi_sensors/data',
            HebiSensors,
            self._hebi_sensor_callback,
            queue_size=None,
        )
        # Initialize feedback message buffers.
        self.hebi_sensors = HebiSensors()
        # Maintain two joint state messages,
        # one in sync with the base TF, other for the latest readings.
        self.update_joint_state = JointState()
        self.pub_joint_state = JointState()

        self.snake_type = rospy.get_param('snake_type', 'REU')

        self.update_joint_state.name = JOINT_NAMES[self.snake_type]
        self.pub_joint_state.name = JOINT_NAMES[self.snake_type]

        self.static_transform_stamped = TransformStamped()
        self.static_transform_stamped.header.frame_id = "map"
        self.static_transform_stamped.child_frame_id = "kdl_dummy_root"

        self.loop_rate = 20
        self.robot = Robot(robot_name=self.snake_type, full_robot=True)

    def run_loop(self):
        r"""Send commands regularly until shutdown."""

        self.update_base_transform()

        # Assemble and publish the data
        stamp = rospy.Time.now()

        self.pub_joint_state.header.stamp = stamp
        self.static_transform_stamped.header.stamp = stamp

        if not hasattr(self, 'pub_joint_angles'):
            self.pub_joint_angles = self.update_joint_state.position
        self.pub_joint_state.position = self.pub_joint_angles

        self.joint_state_pub.publish(self.pub_joint_state)

        broadcaster = StaticTransformBroadcaster()
        broadcaster.sendTransform(self.static_transform_stamped)

    def _hebi_sensor_callback(self, msg: HebiSensors):
        r"""Saves the received sensor data

        Args:
            msg: HebiSensors message containing sensor data.

        """
        self.update_joint_state.position = msg.position
        self.update_joint_state.velocity = msg.velocity
        self.update_joint_state.effort = msg.effort

    def start(self):
        r"""Run loop until ros is not shutdown."""
        rate = rospy.Rate(self.loop_rate)

        while not rospy.is_shutdown():
            self.run_loop()
            rate.sleep()

    def update_base_transform(self):
        r"""Updates the base transform based on the current snake state."""

        translation, rotation = Vector3(0, 0, 0), Quaternion(0, 0, 0, 1)

        if len(self.update_joint_state.position) != 0:
            translation, rotation = self.compute_virtual_chassis()

        self.static_transform_stamped.transform.translation = translation
        self.static_transform_stamped.transform.rotation = rotation

    def compute_virtual_chassis(self):
        r"""Uses Virtual Chassis to update the base transform.

        Reference:
            Rollinson, David, Austin Buchan, and Howie Choset. "Virtual chassis for snake robots: Definition and applications." Advanced Robotics 26.17 (2012): 2043-2064.

        TODO:
            - Make static method, removing dependency on class attributes, allowing usage outside this class.

        """
        self.pub_joint_angles = self.update_joint_state.position
        module_positions = self.robot.fk(self.pub_joint_angles, compute_all_joints=True, invert_base_transform=False)

        # Flip x and y
        temp = module_positions[:, 0].copy()
        module_positions[:, 0] = module_positions[:, 1]
        module_positions[:, 1] = -temp

        module_centers = np.mean(module_positions, axis=-2)

        zero_module_positions = module_positions - module_centers

        _, _, rotation_matrix = np.linalg.svd(zero_module_positions)

        if not hasattr(self, 'rotation_vector_0'):
            self.rotation_vector_0 = rotation_matrix[0]
            self.rotation_vector_1 = rotation_matrix[1]

        rotation_matrix[0] *= np.sign(np.dot(self.rotation_vector_0, rotation_matrix[0]))
        rotation_matrix[1] *= np.sign(np.dot(self.rotation_vector_1, rotation_matrix[1]))
        rotation_matrix[2] = np.cross(rotation_matrix[0], rotation_matrix[1])

        self.rotation_vector_0 = rotation_matrix[0]
        self.rotation_vector_1 = rotation_matrix[1]

        ninety_degree_rotation = np.array([[ 0, 1, 0],
                                            [-1, 0, 0],
                                            [ 0, 0, 1]])

        rotation_matrix = np.matmul(rotation_matrix, ninety_degree_rotation)

        virtual_chassis_rotation = Quaternion(*R.from_matrix(rotation_matrix).as_quat())
        if self.snake_type == 'RSNAKE':
            virtual_chassis_translation = Vector3(module_centers[..., -1], 0, 0)
        elif self.snake_type == 'SEA':
            virtual_chassis_translation = Vector3(-module_centers[..., 1], 0, 0)
        elif self.snake_type == 'REU':
            virtual_chassis_translation = Vector3(module_centers[..., 0], 0, 0)
        else:
            raise ValueError(f'Unknown snake type: {self.snake_type}')
        return virtual_chassis_translation, virtual_chassis_rotation
