#!/usr/bin/env python3

""" 
Run all test scripts under gaitlib_test
"""

from snakelib_test import gaitlib_test
from snakelib_test import reu_pybullet
from snakelib_test import sea_pybullet
from snakelib_test import rsnake_pybullet


def main():

    gaitlib_test.main()
    reu_pybullet.main()
    sea_pybullet.main()
    rsnake_pybullet.main()

    print("All tests complete.")


if __name__ == "__main__":
    main()
