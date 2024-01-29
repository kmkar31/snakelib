import numpy as np

# In future, this structure can allow PyTorch / TensorFlow
def sin(*args, **kwargs):
    return np.sin(*args, **kwargs)

def stack(*args, **kwargs):
    return np.stack(*args, **kwargs)

def compound_serpenoid(self, t,n, params) -> np.ndarray:
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

    if n%2 == 0:
        alpha = params['beta_even'] + params['A_even'] * sin(params['wS_even'] * n- params['wT_even'] * t)
    else:
        alpha = params['beta_odd'] + params['A_odd'] * sin(params['wS_odd'] * n - params['wT_odd'] * t + params['delta'])

    return alpha*(-1)**np.floor(n/2)
