import numpy as np


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

    # 2 separate variables to deal with cases where the number of modules is not multiple of 4
    num_modules_1st_half = int(self.num_modules / 2)
    num_modules_2nd_half = int(self.num_modules / 2)

    # Making first half of the snake longer by 1 in case of odd number of modules
    # This can easily be switched to have second half as longer if we want to
    if self.num_modules % 2 != 0:
        num_modules_1st_half += 1

    # First half of the snake
    target_angles_1 = np.zeros((n_samples, num_modules_1st_half))
    target_angles_1[:] = np.broadcast_to(
        np.array(target_angles_1, dtype=np.float), (n_samples, num_modules_1st_half)
    )

    even_n_1 = np.broadcast_to(
        np.arange(0, num_modules_1st_half, 2, dtype=np.float),
        (
            n_samples,
            int((num_modules_1st_half + 1) / 2),
        ),  # +1 to deal with odd number of modules in first half
    )
    odd_n_1 = np.broadcast_to(
        np.arange(1, num_modules_1st_half, 2, dtype=np.float),
        (n_samples, int(num_modules_1st_half / 2)),
    )

    target_angles_1[:, ::2] = beta_even + A_even * np.sin(
        wS_even * even_n_1 + wT_even * t
    )
    target_angles_1[:, 1::2] = beta_odd + A_odd * np.sin(
        wS_odd * odd_n_1 + wT_odd * t + delta
    )

    # Second half of the snake
    target_angles_2 = np.zeros((n_samples, num_modules_2nd_half))
    target_angles_2[:] = np.broadcast_to(
        np.array(target_angles_2, dtype=np.float), (n_samples, num_modules_2nd_half)
    )

    even_n_2 = np.broadcast_to(
        np.arange(0 + num_modules_1st_half, self.num_modules, 2, dtype=np.float),
        (
            n_samples,
            int((num_modules_2nd_half + 1) / 2),
        ),  # +1 to deal with odd number of modules in second half
    )
    odd_n_2 = np.broadcast_to(
        np.arange(1 + num_modules_1st_half, self.num_modules, 2, dtype=np.float),
        (n_samples, int(num_modules_2nd_half / 2)),
    )

    target_angles_2[:, ::2] = -beta_even + A_even * np.sin(
        wS_even * even_n_2 + wT_even * t
    )
    target_angles_2[:, 1::2] = -beta_odd + A_odd * np.sin(
        wS_odd * odd_n_2 + wT_odd * t + delta
    )

    target_angles = np.concatenate((target_angles_1, target_angles_2), axis=1)

    target_angles = self.flip_axes(target_angles)

    if n_samples == 1:
        target_angles = target_angles.squeeze().tolist()

    return target_angles
