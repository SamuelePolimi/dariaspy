#!/usr/bin/python2.7
"""
Example
1. Set the robot in teaching mode
2. Record the trajectory (in RL the left arm will just drop down, on the real robot the user can move it in
gravity compensation)
3. The arm will be brought to the initial position of the learned movement
4. The movement learnt will be played

Everything will happen in the cartesian space.
"""


from dariaspy.franka_interface import Franka
from dariaspy.trajectory import LoadTrajectory
from dariaspy.movement_primitives import LearnTrajectory, ClassicSpace
from dariaspy.hardware.scales import DymoScale
from dariaspy.observers import ScaleObserver

if __name__ == "__main__":

    franka = Franka()
    scale = DymoScale()
    scale_observer = ScaleObserver(scale)
    refs = scale_observer.get_possible_refs()
    print(scale_observer("DYMO_WEIGHT"))
    print(refs)

