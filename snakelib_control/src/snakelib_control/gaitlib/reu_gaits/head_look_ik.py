import sys
from typing import Dict
import numpy as np

def head_look_ik(self, t: float, current_angles: np.ndarray, robot, params: Dict=None) -> np.ndarray:
    """Controls the end-effector (snake head) in cartesian space using inverse jacobian approach.

    Args:
        t: current robot time
        robot: robot model object to get the jacobian
        current angles: latest joint angles readings
        params: update (if any) in parameters
    """
    n_headlook_modules = 6 # number of modules in the headlook group.

    self.current_gait = 'head_look'
    # Update the current parameters if params is not empty
    params = {} if params is None else params

    params = self.update_params(
        self.default_gait_params.get(self.current_gait, {}), params
    )

    yaw, y, z = params['x_state'], params['y_state'], params['z_state']
    roll, pitch, x = params['roll'], params['pitch'], params['yaw']

    # delta = np.array([x, y, z, roll, pitch, yaw]) * 0.1
    delta = np.array([roll, pitch, yaw, x, y, z]) * np.array([0.3, 0.3, 0.3, 0.3, 0.3, 0.3])
    # delta = np.array([x, y, z, roll, pitch, yaw]) * params['delta_scale']
    head_look_group_joint_angles = robot.update_rect(delta, clip='norm')[:n_headlook_modules]
    target_angles = np.concatenate((head_look_group_joint_angles, current_angles[n_headlook_modules:]))

    return target_angles
