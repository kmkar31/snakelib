import numpy as np
from copy import deepcopy
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

    N = self.num_modules
    alpha = np.zeros(N)

    gait_params = deepcopy(self.current_gait_params)
    
    for n in range(N):
        gait_params['A_odd'] = n*self.current_gait_params['A_odd']/N
        alpha[n] = self.compound_serpenoid(t, n, gait_params)

    return alpha
