from typing import Dict

import numpy as np
from .utils import make_module_numbers, flip_axes

# In future, this structure can allow PyTorch / TensorFlow
def sin(*args, **kwargs):
    return np.sin(*args, **kwargs)

def stack(*args, **kwargs):
    return np.stack(*args, **kwargs)

def compound_serpenoid(self, t, params) -> np.ndarray:
    r"""Vectorized implementation of the compound serpenoid equation

    ...math::
        \alpha(n, t) = \beta_{even} + A_{even}sin(\omega_{t, even}t + \omega_{s, even}n) \forall even n \\
        \alpha(n, t) = \beta_{odd} + A_{odd}sin(\omega_{t, odd} + \omega_{s, odd}n) \forall odd n

    Args:
        params (dict): Dictionary of gait parameters
        t (float): Time
        N (int): Number of modules

    Returns:
        alpha (np.ndarray): Array of target angles for each module
    """
    N = self.num_modules
    n_samples = 1 if (isinstance(t, float) or isinstance(t, int)) else len(t)

    even_n = make_module_numbers(N, n_samples, even=True)
    odd_n = make_module_numbers(N, n_samples, even=False)

    alpha_even = params['beta_even'] + params['A_even'] * sin(params['wS_even'] * even_n - params['wT_even'] * t)
    alpha_odd = params['beta_odd'] + params['A_odd'] * sin(params['wS_odd'] * odd_n - params['wT_odd'] * t + params['delta'])

    alpha_stacked = stack([alpha_even, alpha_odd], axis=-1)
    alpha = alpha_stacked.reshape(*alpha_stacked[:-2], -1)

    return flip_axes(alpha)
