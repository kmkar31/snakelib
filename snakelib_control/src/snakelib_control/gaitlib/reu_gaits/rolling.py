import numpy as np

def rolling(self, t=0, params=None):

    params = {} if params is None else params

    self.current_gait = "rolling"

    # Update the current parameters if params is not empty
    self.current_gait_params = self.update_params(
        self.default_gait_params.get(self.current_gait), params
    )

    # Math formulation of rolling is identical to sidewinding
    N = self.num_modules
    alpha = np.zeros(N)

    for n in range(N):
        alpha[n] = self.compound_serpenoid(t, n, self.current_gait_params)

    return alpha
