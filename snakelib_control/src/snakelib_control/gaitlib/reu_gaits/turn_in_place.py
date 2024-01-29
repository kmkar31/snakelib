import numpy as np
from copy import deepcopy

def turn_in_place(self, t=0, params=None):

    params = {} if params is None else params

    self.current_gait = "turn_in_place"

    # Update the current parameters if params is not empty
    self.current_gait_params = self.update_params(
        self.default_gait_params.get(self.current_gait), params
    )

    N = self.num_modules
    alpha = np.zeros(N)

    gait_params = deepcopy(self.current_gait_params)

    for n in range(N):
        if n <= np.floor(N/2):
            gait_params['delta'] = -self.current_gait_params['delta']
            alpha[n] = self.compound_serpenoid(t, n, gait_params)
        else:
            alpha[n] = self.compound_serpenoid(t, n, self.current_gait_params)
    
    return alpha
