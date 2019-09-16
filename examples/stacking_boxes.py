"""
Example - Simulating a stacking task
"""

import time

from dariaspy.ros_listener import activate_listener
from dariaspy.darias_interface import Darias, DariasMode
from dariaspy.positions import  Home_Position
from dariaspy.observers import DariasObserver
from dariaspy.recording import Recorder
from dariaspy.trajectory import GoToTrajectory, LoadTrajectory


def move(obs, refs, displacement=0.1):
    for ref in refs:
        obs[ref] += displacement
    return obs


def go_home():
    darias.go_to(GoToTrajectory(duration=0.1, **Home_Position), "RIGHT_HAND")


def open_hand():
    trajectory = LoadTrajectory("positions/open_hand.npy")
    darias.go_to(trajectory, "RIGHT_HAND")


def record_traj(obs):
    darias.kinesthetic("RIGHT_ARM")
    rec = Recorder(obs, darias.groups["RIGHT_ARM"].refs, sampling_frequency=5)
    rec.record_fixed_duration(15.)
    return rec.trajectory


def grasp():
    trajectory = LoadTrajectory("positions/grasp.npy")
    darias.go_to(trajectory, "RIGHT_HAND")


#TODO: comment the code and print the useful information
if __name__ == "__main__":

    activate_listener()

    darias = Darias()
    pick_trajectories = []
    place_trajectories = []
    pull_arm = None

    observer = DariasObserver(darias)

    darias.go_to(GoToTrajectory(**Home_Position), "RIGHT_ARM")
    n_boxes = 4
    for _ in range(n_boxes):
        open_hand()
        print("go over box")
        pick_trajectories.append(record_traj(observer))
        grasp()
        print("build tower")
        place_trajectories.append(record_traj(observer))

    open_hand()
    pull_arm = record_traj(observer)
    darias.mode.set_mode(DariasMode.DariasCommandMode)

    for i in range(15):
        time.sleep(1)
        print(10 - i)


    darias.go_to(GoToTrajectory(**Home_Position), "RIGHT_ARM")
    n_boxes = 4
    for pick, place in zip(pick_trajectories, place_trajectories):
        open_hand()
        print("go over box")
        darias.go_to(pick, "RIGHT_ARM")
        grasp()
        print("build tower")
        darias.go_to(place, "RIGHT_ARM")

    open_hand()
    darias.go_to(pull_arm, "RIGHT_ARM")
    darias.kinesthetic("RIGHT_ARM")




