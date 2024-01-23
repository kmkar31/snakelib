from typing import Dict

import numpy as np

# TODO: get rid of these magic numbers.
JOINT_TOLERANCE = 0.15 # tolerance for joint angles
MIN_JOINT_ANGLE = -np.pi/2.0 + JOINT_TOLERANCE
MAX_JOINT_ANGLE = np.pi/2.0 - JOINT_TOLERANCE
dt = 0.025

def head_look(self, t: float=0, current_angles: np.ndarray=None, params: Dict=None) -> np.ndarray:
    """
    Args:
        t: current robot time
        current angles: latest joint angles readings
        params: update (if any) in parameters
    """
    n_headlook_modules = 2 # number of modules in the headlook group.

    self.current_gait = 'head_look'
    # Update the current parameters if params is not empty
    params = {} if params is None else params

    params = self.update_params(
        self.default_gait_params.get(self.current_gait, {}), params
    )

    x_state, y_state = params['x_state'], params['y_state']

    # Use accelerations to correct direction of rotation.
    head_acc = params['head_acc']
    before_head_vel = y_state if abs(head_acc[1]) > abs(head_acc[0]) else x_state
    major_comp_sign = np.sign(head_acc[np.argmax(np.abs(head_acc))])
    before_head_vel *= major_comp_sign
    head_vel =  x_state if abs(head_acc[1]) > abs(head_acc[0]) else y_state
    head_vel *= -np.sign(head_acc[np.argmax(np.abs(head_acc))])
    if abs(head_acc[1]) > abs(head_acc[0]):
        head_vel *= -1

    vel = np.array([head_vel, before_head_vel])

    # Slice head group joint angles form all joint angles.
    headlook_group_current_angles = current_angles[:n_headlook_modules]

    headlook_group_target_angles = headlook_group_current_angles + (vel * dt)
    headlook_group_target_angles = np.clip(headlook_group_target_angles, MIN_JOINT_ANGLE, MAX_JOINT_ANGLE)
    target_angles = np.concatenate((headlook_group_target_angles, current_angles[n_headlook_modules:]))

    return target_angles
