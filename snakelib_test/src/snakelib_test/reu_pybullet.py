"""
Test that runs through all gaits of ReU snake using PyBullet
"""

from matplotlib.pyplot import angle_spectrum
import snake_bullet.snake_bullet as snake_bullet
import gaitlib.reu_gaits as reu_gaits
import numpy as np


def main():

    max_steps = 300
    t = 0
    dt = 0.01

    reu = reu_gaits.ReuGaits()
    snake_type = reu.snake_type

    snake_sim = snake_bullet.SnakeBullet(snake_type)

    for step in range(max_steps):

        joint_angles = reu.lateral_undulation(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = reu.linear_progression(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = reu.rolling_helix(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = reu.rolling(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = reu.conical_sidewinding(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = reu.slithering(t)
        snake_sim.step(joint_angles)
        t = t + dt

    for step in range(max_steps):

        joint_angles = reu.turn_in_place(t)
        snake_sim.step(joint_angles)
        t = t + dt


if __name__ == "__main__":
    main()
