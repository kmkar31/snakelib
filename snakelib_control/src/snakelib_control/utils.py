import os
import sys
from typing import Dict
from warnings import warn

import numpy as np
from pykdl_utils.kdl_kinematics import create_kdl_kin

ROBOT_CFGS = dict(
    urdf_path=dict(
        REU='snakelib_description/REU_snake/REU_snake.urdf',
        SEA='snakelib_description/SEA_snake/SEA_snake.urdf',
        RSNAKE='snakelib_description/RSNAKE_snake/RSNAKE_snake.urdf',
    ),
    base_link=dict(
        REU='kdl_dummy_root',
        SEA='kdl_dummy_root',
        RSNAKE='kdl_dummy_root',
    ),
    end_link=dict(
        REU='reu_module_out_16',
        SEA='tail',
        RSNAKE='module_14',
        headlook_ik=dict(
            REU='reu_module_out_6',
            SEA='SA007__MoJo__INPUT_INTERFACE',
            RSNAKE='module_6',
        )
    ),
    start_angles=[0.9, 1.2, 0.5, 1.2, -1.2, -0.9], # initial angles of the headlook
    max_joint_angle_deltas_norm=0.02, # So that the Jacobian used to update the angles is reliable
    max_joint_angle=1.5, # Slightly lower than pi/2 to avoid joint limits
    min_joint_angles_abs=0.0, # Slightly higher than 0 to avoid singularities
    )

JOINT_NAMES = {
    'REU': [f'reu_joint_out2in_{i}' for i in range(0, 16)],
    'SEA': [f'SA00{i}__MoJo' for i in range(1, 10)] + [f'SA0{i}__MoJo' for i in range(10, 17)],
    'RSNAKE': [f'module_{i}_to_{i+1}' for i in range(14)],
}

LINK_NAMES = {
    'REU': [f'reu_module_out_{i}' for i in range(17)],
    'SEA': [f'SA00{i}__MoJo__INPUT_INTERFACE' for i in range(1, 10)] + [f'SA0{i}__MoJo__INPUT_INTERFACE' for i in range(10, 17)],
    'RSNAKE': [f'module_{i}' for i in range(15)],
}

class Robot:
    """Wrapper class for interacting with the robot URDF and KDL kinematics"""
    def __init__(self,
                 cfgs: Dict=ROBOT_CFGS,
                 robot_name: str='REU',
                 full_robot: bool=False):
        """Constructs a Robot object from specified configurations

        Args:
            cfgs (Dict, optional): Specifications like urdf path, link names, and joint names. Defaults to ReU_cfgs.
            robot_name (str, optional): Robot type to initialize.
            full_robot (bool, optional): Initialize all links of just some of them.
        """

        self.urdf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../', cfgs['urdf_path'][robot_name]))
        self.links = LINK_NAMES[robot_name]
        self.base_link = cfgs['base_link'][robot_name]
        if full_robot:
            self.end_link = cfgs['end_link'][robot_name]
        else:
            self.end_link = cfgs['end_link']['headlook_ik'][robot_name]
        self.kdl_kin = create_kdl_kin(self.base_link, self.end_link, urdf_filename=self.urdf_path)

        self.prev_angles = np.array(cfgs['start_angles'])

        self.max_joint_angle_deltas_norm = cfgs['max_joint_angle_deltas_norm']
        self.max_joint_angle = cfgs['max_joint_angle']
        self.min_joint_angles_abs = cfgs['min_joint_angles_abs']

    def update_rect(self, delta: np.ndarray, dt: float=0.016, clip: str='norm'):
        """Updates the robot's joint angles based on the cartesian velocities (delta)

        Args:
            delta (np.ndarray): velocities in cartesian space
            dt (float, optional): difference in subsequent timesteps. Defaults to 0.016.
            clip (str, optional): clipping method for joint angle deltas. Defaults to 'norm'.
        """
        joint_vels = self.delta_to_joint_vels(delta)
        joint_angle_deltas = joint_vels * dt

        # Instead of clipping individual joint angle deltas, clip the norm of the joint angle deltas
        # to preserve the direcition of the joint angle deltas vector
        if clip == 'norm':
            joint_angle_deltas_norm = np.linalg.norm(joint_angle_deltas)
            factor = max(1.0 , joint_angle_deltas_norm / self.max_joint_angle_deltas_norm)
            joint_angle_deltas /= factor
        # Clip abs values of individual deltas
        elif clip == 'individual':
            joint_angle_deltas = np.clip(joint_angle_deltas, -self.max_joint_angle_delta, self.max_joint_angle_delta)
        else:
            joint_angle_deltas = joint_angle_deltas

        joint_angles = np.clip(self.prev_angles + joint_angle_deltas, -self.max_joint_angle, self.max_joint_angle)
        self.prev_angles = joint_angles = np.sign(joint_angles) * np.maximum(np.abs(joint_angles), self.min_joint_angles_abs)
        return joint_angles

    def delta_to_joint_vels(self, delta: np.ndarray):
        """Converts velocities (delta) in cartesian space to joint velocities

        Args:
            delta (np.ndarray): velocities in cartesian space

        Returns:
            joint_vels (np.ndarray): Joint velocities
        """
        Jacobian = self.kdl_kin.jacobian(self.prev_angles)
        Jacobian_inv = np.linalg.pinv(Jacobian)
        joint_vels = Jacobian_inv.dot(delta)
        return np.array(joint_vels).squeeze()

    def fk(self,
           joint_angles: np.ndarray,
           compute_all_joints: bool=False,
           invert_base_transform: bool=True,):
        """Computes forward kinematics

        Args:
            joint_angles (np.ndarray): Joint angles
            compute_all_joints (bool, optional): If True, computes the forward kinematics for all links. Defaults to False.
            invert_base_transform (bool, optional): Only turn on if base transform is Identity. Defaults to True.

        Returns:
            poses (np.ndarray): Positions of the requested links
        """
        if compute_all_joints:
            poses = []
            for idx, link in enumerate(self.links[1:]):
                if invert_base_transform:
                    pose = self.kdl_kin.forward(joint_angles, end_link=link, base_link=self.base_link)
                else:
                    pose = self.forward_dont_invert_base(joint_angles, end_link=link, base_link=self.base_link)
                poses.append(np.squeeze(np.asarray(pose))[:3, 3])
        else:
            pose = self.kdl_kin.forward(joint_angles, end_link=self.end_link, base_link=self.base_link)
            poses = [np.squeeze(np.asarray(pose))[:3, 3]]
        return np.stack(poses)

    def forward_dont_invert_base(self, q, end_link=None, base_link=None):
        """Copied from pykdl utils except inverting base_trans"""
        link_names = self.kdl_kin.get_link_names()
        if end_link is None:
            end_link = self.kdl_kin.chain.getNrOfSegments()
        else:
            end_link = end_link.split("/")[-1]
            if end_link in link_names:
                end_link = link_names.index(end_link)
            else:
                print (f'Target segment {end_link} not in KDL chain')
                return None
        if base_link is None:
            base_link = 0
        else:
            base_link = base_link.split("/")[-1]
            if base_link in link_names:
                base_link = link_names.index(base_link)
            else:
                print(f'Base segment {base_link} not in KDL chain')
                return None
        base_trans = self.kdl_kin._do_kdl_fk(q, base_link)
        if base_trans is None:
            print(f'FK KDL failure on base transformation')
        end_trans = self.kdl_kin._do_kdl_fk(q, end_link)
        if end_trans is None:
            print(f'FK KDL failure on end transformation.')
        return base_trans * end_trans

    def ik(self, pose: np.ndarray, guess: np.ndarray=None):
        """Computes inverse kinematics for the desired end effector pose

        Args:
            pose (np.ndarray): Desired pose
            guess (np.ndarray, optional): Initial guess for the inverse kinematics. Defaults to None.

        Returns:
            self.prev_angles (np.ndarray): Joint angles (if IK is successful) or previous joint angles (if IK fails)
        """
        warn('ik is not tested')
        self.prev_pose = pose
        update_angles = self.inverse_search(pose, q_init=guess, max_iters=300)
        if update_angles is not None:
            self.prev_angles = update_angles
        else:
            print('falling back to prev')
        return self.prev_angles

    def inverse_search(self,
                       pos: np.ndarray,
                       q_init: np.ndarray=None,
                       max_iters: int=100,
                       min_joints: np.ndarray=None,
                       max_joints: np.ndarray=None):
        """Searches for a valid inverse kinematics solution

        Args:
            pos (np.ndarray): Desired pose
            q_init (np.ndarray, optional): Initial guess for the inverse kinematics. Defaults to None.
            max_iters (int, optional): Maximum number of iterations to search IK solution. Defaults to 100.
            min_joints (np.ndarray, optional): Minimum limit of the IK solution. Defaults to None.
            max_joints (np.ndarray, optional): Maximum limit of the IK solution. Defaults to None.

        Returns:
            q_ik (np.ndarray): Joint angles if IK is successful, None otherwise
        """
        warn('inverse_search is not tested')
        iters = 0
        if min_joints is None:
            min_joints = self.kdl_kin.joint_safety_lower
        if max_joints is None:
            max_joints = self.kdl_kin.joint_safety_upper
        # while not rospy.is_shutdown() and rospy.get_time() - st_time < timeout:
        q_ik = None
        while (q_ik is None) and (iters < max_iters):
            if q_init is None:
                q_init = self.kdl_kin.random_joint_angles()
            q_ik = self.kdl_kin.inverse(pos, q_init, min_joints, max_joints)
            if q_ik is not None:
                return q_ik
            q_init = None
            iters += 1
        return None