import numpy as np
import matplotlib.pyplot as plt
from .utils import make_module_numbers

def conical_sidewinding(self, t=0, params=None):

    params = {} if params is None else params

    self.current_gait = "conical_sidewinding"

    # Update the current parameters if params is not empty
    self.current_gait_params = self.update_params(
        self.default_gait_params.get(self.current_gait), params
    )

    # number of time samples
    n_samples = 1 if (isinstance(t, float) or isinstance(t, int)) else len(t)

    # Extract individual gait parameters from dictionary
    beta_even = self.current_gait_params['beta_even']
    beta_odd = self.current_gait_params['beta_odd']
    A_even_nom = self.current_gait_params['A_even']
    A_even_tol = self.current_gait_params['A_even_tol']
    A_odd_nom = self.current_gait_params['A_odd']
    A_odd_tol = self.current_gait_params['A_odd_tol']
    wS_even = self.current_gait_params['wS_even']
    wS_odd = self.current_gait_params['wS_odd']
    wT_even = self.current_gait_params['wT_even']
    wT_odd = self.current_gait_params['wT_odd']
    delta = self.current_gait_params['delta']
    slope = self.current_gait_params.get('slope', 0)

    # Initialize and reshape based on number of time samples
    target_angles = np.zeros((n_samples, self.num_modules))
    target_angles[:] = np.broadcast_to(
        np.array(target_angles, dtype=np.float), (n_samples, self.num_modules)
    )

    even_n = make_module_numbers(self.num_modules, n_samples, even=True)
    odd_n = make_module_numbers(self.num_modules, n_samples, even=False)

    zero_centered_module_indexes_even = np.arange((self.num_modules + 1)//2) - (self.num_modules + 1)//4
    zero_centered_module_indexes_odd = np.arange(self.num_modules//2) - self.num_modules//4

    A_even_raw = A_even_nom + slope * zero_centered_module_indexes_even
    A_even = np.clip(A_even_raw, A_even_nom - A_even_tol, A_even_nom + A_even_tol) # (num_modules,)

    A_odd_raw = A_odd_nom + slope * zero_centered_module_indexes_odd
    A_odd = np.clip(A_odd_raw, A_odd_nom - A_odd_tol, A_odd_nom + A_odd_tol) # (num_modules,)

    target_angles[:, ::2] = beta_even + A_even * np.sin(wS_even * even_n - wT_even * t)
    target_angles[:, 1::2] = beta_odd + A_odd * np.sin(
        wS_odd * odd_n - wT_odd * t + delta
    )

    target_angles = self.flip_axes(target_angles)

    if n_samples == 1:
        target_angles = target_angles.squeeze().tolist()

    return target_angles

