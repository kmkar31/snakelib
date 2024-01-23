"""
Test that runs through all gaits of Rsnake using PyBullet
"""

from matplotlib.pyplot import angle_spectrum
import snake_bullet.snake_bullet as snake_bullet
import gaitlib.rsnake_gaits as rsnake_gaits
import numpy as np


def main():

    max_steps = 300
    t = 0
    dt = 0.01

    r = rsnake_gaits.RsnakeGaits()
    snake_type = r.snake_type
    num_modules = r.num_modules

    snake_sim = snake_bullet.SnakeBullet(
        snake_type=snake_type, num_of_modules=num_modules
    )

    for step in range(max_steps):

        joint_angles = r.lateral_undulation(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = r.linear_progression(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = r.rolling_helix(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = r.rolling(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = r.conical_sidewinding(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = r.slithering(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = r.turn_in_place(t)
        snake_sim.step(joint_angles)
        t = t + dt


if __name__ == "__main__":
    main()
