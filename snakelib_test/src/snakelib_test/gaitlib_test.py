#!/usr/bin/env python3

"""
Script testing for errors and basic functionality of Gaitlib
"""

import gaitlib.reu_gaits as reu_gaits
import gaitlib.sea_gaits as sea_gaits


def main():

    reu = reu_gaits.ReuGaits()
    angles = reu.lateral_undulation(1.1)
    angles = reu.rolling_helix(1.1)
    angles = reu.rolling(1.1)
    angles = reu.conical_sidewinding(1.1)
    angles = reu.slithering(1.1)
    angles = reu.turn_in_place(1.1)

    sea = sea_gaits.ReuGaits()
    angles = sea.lateral_undulation(1.1)
    angles = sea.rolling_helix(1.1)
    angles = sea.rolling(1.1)
    angles = sea.conical_sidewinding(1.1)
    angles = sea.slithering(1.1)
    angles = sea.turn_in_place(1.1)

    print("Gaitlib test complete.")


if __name__ == "__main__":
    main()
