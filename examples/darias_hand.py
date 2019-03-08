"""
Example - Bring the robot in its home position
"""
from dariaspy.darias_interface import Darias
from dariaspy.positions import Home_Position
from dariaspy.observers import DariasObserver
from dariaspy.trajectory import GoToTrajectory, LoadTrajectory


def move(obs, refs, displacement=0.1):
    for ref in refs:
        obs[ref] += displacement
    return obs


def go_home():
    darias.go_to(GoToTrajectory(duration=0.1, **Home_Position), "RIGHT_HAND")


def open_hand():
    trajectory = LoadTrajectory("positions/open_hand_2.npy")
    darias.go_to(trajectory, "RIGHT_HAND")


def grasp():
    trajectory = LoadTrajectory("positions/grasp.npy")
    darias.go_to(trajectory, "RIGHT_HAND")


if __name__ == "__main__":

    darias = Darias()

    observer = DariasObserver(darias)
    observation = observer(*darias.groups["RIGHT_HAND"].refs)

    open_hand()

    grasp()

    open_hand()



