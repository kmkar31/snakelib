import numpy as np
from .utils import make_module_numbers

def slithering(self, t=0, params=None):

    params = {} if params is None else params

    self.current_gait = "slithering"

    # Update the current parameters if params is not empty
    self.current_gait_params = self.update_params(
        self.default_gait_params.get(self.current_gait), params
    )

    """
    We do not reuse _sidewinding because slithering has a unique way of defining 
    amplitude for the odd joints.
    """
    # number of time samples
    n_samples = 1 if (isinstance(t, float) or isinstance(t, int)) else len(t)

    # Extract individual gait parameters from dictionary
    beta_even = self.current_gait_params['beta_even']
    beta_odd = self.current_gait_params['beta_odd']
    A_even = self.current_gait_params['A_even']
    A_odd = self.current_gait_params['A_odd']
    wS_even = self.current_gait_params['wS_even']
    wS_odd = self.current_gait_params['wS_odd']
    wT_even = self.current_gait_params['wT_even']
    wT_odd = self.current_gait_params['wT_odd']
    delta = self.current_gait_params['delta']

    # Initialize and reshape based on number of time samples
    target_angles = np.zeros((n_samples, self.num_modules))
    target_angles[:] = np.broadcast_to(
        np.array(target_angles, dtype=np.float), (n_samples, self.num_modules)
    )

    even_n = make_module_numbers(self.num_modules, n_samples, even=True)
    odd_n = make_module_numbers(self.num_modules, n_samples, even=False)

    target_angles[:, ::2] = beta_even + A_even * np.sin(wS_even * even_n - wT_even * t)
    target_angles[:, 1::2] = beta_odd + (odd_n) * (A_odd / self.num_modules) * np.sin(
        wS_odd * odd_n - wT_odd * t + delta
    )

    target_angles = self.flip_axes(target_angles)

    if n_samples == 1:
        target_angles = target_angles.squeeze().tolist()

    return target_angles
