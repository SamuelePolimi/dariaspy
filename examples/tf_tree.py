#!/usr/bin/env python
"""
Example - TF-Tree. Working in progress!
"""
import time

from dariaspy.optitrack import TFFrames
from dariaspy.darias_interface import Darias


if __name__ == "__main__":
    t = TFFrames()
    goal = t.get_frame("ENDEFF_LEFT_ARM")
    print(goal.duration)
    print(goal.position)
