"""
Test that runs through all gaits of SEA snake using PyBullet
"""

from matplotlib.pyplot import angle_spectrum
import snake_bullet.snake_bullet as snake_bullet
import gaitlib.sea_gaits as sea_gaits
import numpy as np


def main():

    max_steps = 300
    t = 0
    dt = 0.01

    sea = sea_gaits.SeaGaits()
    snake_type = sea.snake_type

    snake_sim = snake_bullet.SnakeBullet(snake_type)

    for step in range(max_steps):

        joint_angles = sea.lateral_undulation(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = sea.linear_progression(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = sea.rolling_helix(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = sea.rolling(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = sea.conical_sidewinding(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = sea.slithering(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = sea.turn_in_place(t)
        snake_sim.step(joint_angles)
        t = t + dt


if __name__ == "__main__":
    main()
