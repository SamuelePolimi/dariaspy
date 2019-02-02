"""
Example - Bring the robot in its home position
"""
import subprocess
from sound_play.libsoundplay import SoundClient
import time
import os
import speech_recognition as sr

from dariaspy.darias_interface import Darias, DariasMode
from dariaspy.positions import Home_Right_Joints, Home_Left_Joints, Home_Position
from dariaspy.darias_space import Trajectory, JointGoal
from dariaspy.observers import DariasObserver
from dariaspy.recording import Recorder
from dariaspy.trajectory import GoToTrajectory, LoadTrajectory


def move(obs, refs, displacement=0.1):
    for ref in refs:
        obs[ref] += displacement
    return obs


if __name__ == "__main__":

    darias = Darias()

    default_group = "WHOLE_ROBOT"

    darias.go_to_1(GoToTrajectory(duration=10., **Home_Position), "LEFT_HAND")
    darias.go_to_1(LoadTrajectory("traj.npy"), "LEFT_HAND")
    exit()
    observer = DariasObserver(darias)
    observation = observer(*darias.groups["LEFT_HAND"].refs)
    for _ in range(10):
        observation = move(observation, ['L_RIP', 'L_MIP', 'L_INP', 'L_RIP', 'L_SMP'], displacement=0.05)
        darias.go_to_1(GoToTrajectory(duration=0.1, **observation), "LEFT_HAND")


    fin_traj = GoToTrajectory(duration=1., **observation)
    fin_traj.save("traj.npy")



