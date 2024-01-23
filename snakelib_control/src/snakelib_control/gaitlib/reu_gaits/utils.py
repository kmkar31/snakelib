import numpy as np

def make_module_numbers(N: int, n_samples: int, even: bool=True):
    """Creates an array of joint numbers

    Args:
        N: number of modules
        n_samples: number of joint number arrays to generate

    Returns:
        joint_numbers (n_samples, N): 2D array of joint numbers
    """
    offset = 0 if even else 1
    L = int((N + 1) / 2) if even else int(N / 2)

    joint_numbers = np.broadcast_to(
        np.arange(offset, N, 2, dtype=np.float),
        (n_samples, L,),  # +1 to deal with odd number of modules
    )

    return joint_numbers

def flip_axes(target_angles: np.ndarray):
    """The direction of the target angles has to flip signs after every pair of actuators,
    since on ReU the rotation axes of subsequent even or odd joints is flipped as
    the robot is assembled.
    """
    flip_pattern = [1, 1, -1, -1]
    N = target_angles.shape[-1]
    full_pattern = np.tile(flip_pattern, int(np.ceil(N / 4)))[:N]
    return target_angles * full_pattern
