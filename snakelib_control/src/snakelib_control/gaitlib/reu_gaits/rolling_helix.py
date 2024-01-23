import numpy as np

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

    # TODO: the section below should be updated to reuse code from _sidewinding.
    # Update the gait parameters and then provie that to _sidewinding.

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
    wt_direction = self.current_gait_params['wt_direction']
    tightness = self.current_gait_params['tightness']
    pole_direction = self.current_gait_params['pole_direction']

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

    wS_max = wS_even # Assume wS_even to be wS_max from yaml.
    A_min = A_even

    # Update spatial frequency using commanded tightness.
    if tightness < A_transition:
        wS_odd = 0
    else:
        wS_odd = min(wS_max, (tightness - A_transition) * dWs_dAodd)

    wS_odd *= -pole_direction

    # Update amplitude using commanded tightness.
    if tightness < A_min:
        A_odd = A_min
    else:
        A_odd = min(tightness, A_max)

    A_odd *= -pole_direction

    wS_even = wS_odd
    A_even = A_odd

    # Initialize and reshape based on number of time samples
    target_angles = np.zeros((n_samples, self.num_modules))
    target_angles[:] = np.broadcast_to(
        np.array(target_angles, dtype=np.float), (n_samples, self.num_modules)
    )

    even_n = np.broadcast_to(
        np.arange(0, self.num_modules, 2, dtype=np.float),
        (
            n_samples,
            int((self.num_modules + 1) / 2),
        ),  # +1 to deal with odd number of modules
    )
    odd_n = np.broadcast_to(
        np.arange(1, self.num_modules, 2, dtype=np.float),
        (n_samples, int(self.num_modules / 2)),
    )

    target_angles[:, ::2] = beta_even + A_even * np.sin(wS_even * even_n + wT_even * t + np.pi)
    target_angles[:, 1::2] = beta_odd + A_odd * np.sin(
        wS_odd * odd_n + wT_odd * t + delta
    )

    target_angles = self.flip_axes(target_angles)

    if n_samples == 1:
        target_angles = target_angles.squeeze().tolist()

    return target_angles
