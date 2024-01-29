import numpy as np
from copy import deepcopy

def linear_progression(self, t=0, params=None):

    params = {} if params is None else params

    self.current_gait = "linear_progression"

    # Update the current parameters if params is not empty
    self.current_gait_params = self.update_params(
        self.default_gait_params.get(self.current_gait), params
    )

    """
    Linear progression does not reuse _sidewinding because the sinusoids are propagated
    in different directions for the front and back halves of the snake
    """
    
    N = self.num_modules
    alpha = np.zeros(N)

    second_half_params = deepcopy(self.current_gait_params)
    second_half_params['beta_even'] = -second_half_params['beta_even']
    second_half_params['beta_odd'] = -second_half_params['beta_odd']

    for n in range(N):
        if n <= np.floor(N/2):
            alpha[n] = self.compound_serpenoid(t, n, self.current_gait_params)
        else:
            alpha[n] = self.compound_serpenoid(t, n, second_half_params)
    
    return alpha
