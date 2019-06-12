#!/usr/bin/env python

"""
Contains variosu functions that may be useful by multiple plugins in the GUI
Contact: Bob DeBortoli (debortor@oregonstate.edu)

Copyright Carnegie Mellon University / Oregon State University <2019>
This code is proprietary to the CMU SubT challenge. Do not share or distribute without express permission of a project lead (Sebastion or Matt).
"""


def displaySeconds(seconds):
    """Function to convert seconds, which should be a float, into a min:sec string."""
    return "%02d:%02d" % (seconds / 60, seconds % 60)


def rgb(r, g, b):
    return "background-color:rgb({0}, {1}, {2})".format(r, g, b)


class COLORS:
    GRAY = rgb(126, 126, 126)
    DARK_GRAY = rgb(100, 100, 100)
    RED = rgb(220, 0, 0)
    GREEN = rgb(0, 220, 0)
    BLUE = rgb(0, 100, 220)
    ORANGE = rgb(255, 130, 0)
