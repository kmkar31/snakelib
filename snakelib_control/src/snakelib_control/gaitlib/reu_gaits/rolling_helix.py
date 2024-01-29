import numpy as np
from copy import deepcopy

def rolling_helix(self, t=0, params=None, pole_params=None):
    params = {} if params is None else params
    pole_params = {} if pole_params is None else pole_params

    self.current_gait = "rolling_helix"

    # Update the current parameters if params is not empty
    self.current_gait_params = self.update_params(
        self.default_gait_params.get(self.current_gait), params
    )

    A_transition = pole_params.get("A_transition", 0.35)
    A_max = pole_params.get("A_max", 1.25)
    dWs_dAodd = pole_params.get("dWs_dAodd", 2.5 / 0.75)

    gait_params = deepcopy(self.current_gait_params)
    """ We start with spatial frequency being zero. Once the amplitude reaches
        amplitude_transition, we then set spatial frequency according to following line:
             ^
             |                     * * * * * * * * wS_odd
             |                   *
             |     dWs_dAodd   *
             |       --------*
         wS  |       |     *
             |       |   *
             |       | *
             |       *
    wS_min-->|     *
             |     *
             * * * *------------------------------>
                   ^           A_odd
                   |
                A_transition

    The "tightness" of the helix is therefore determined solely by the amplitude. It is
    also possible to adjust wS and the amplitude directly by deriving the Jacobian relating
    the radius of the helix and wS/amplitude. We leave this for future work.
    """

    wS_max = gait_params["wS_even"] # Assume wS_even to be wS_max from yaml.
    A_min = gait_params["A_even"]

    # Update spatial frequency using commanded tightness.
    if gait_params["tightness"] < A_transition:
        gait_params["wS_odd"] = 0
    else:
        gait_params["wS_odd"] = min(wS_max, (gait_params["tightness"] - A_transition) * dWs_dAodd)

    gait_params["wS_odd"] *= -gait_params["pole_direction"]

    # Update amplitude using commanded tightness.
    if gait_params["tightness"] < A_min:
        gait_params["A_odd"] = A_min
    else:
        gait_params["A_odd"] = min(gait_params["tightness"], A_max)

    gait_params["A_odd"] *= -gait_params["pole_direction"]

    gait_params["wS_even"] = gait_params["wS_odd"]
    gait_params["A_even"] = gait_params["A_odd"]

    N = self.num_modules

    alpha = np.zeros(N)
    for n in range(N):
        alpha[n] = self.compound_serpenoid(t,n,gait_params)
    
    return alpha


