"""
Example - Bring the robot in its home position
"""
import subprocess
from sound_play.libsoundplay import SoundClient
import time
import os
#import speech_recognition as sr

from dariaspy.darias_interface import Darias, DariasMode
from dariaspy.positions import Home_Right_Joints, Home_Left_Joints, Home_Position
from dariaspy.darias_space import Trajectory, JointGoal
from dariaspy.observers import DariasObserver, EndEffectorObserver
from dariaspy.recording import Recorder
from dariaspy.trajectory import GoToTrajectory, LoadTrajectory


def move(obs, refs, displacement=0.1):
    for ref in refs:
        obs[ref] += displacement
    return obs


def go_home():
    darias.go_to_1(GoToTrajectory(duration=0.1, **Home_Position), "RIGHT_HAND")


def open_hand():
    trajectory = LoadTrajectory("open_hand.npy")
    darias.go_to_1(trajectory, "RIGHT_HAND")


def record_traj(obs):
    darias.kinesthetic("RIGHT_ARM")
    rec = Recorder(obs, darias.groups["RIGHT_ARM"].refs, sampling_frequency=5)
    rec.record_fixed_duration(15.)
    return rec.trajectory


def grasp():
    trajectory = LoadTrajectory("grasp.npy")
    darias.go_to_1(trajectory, "RIGHT_HAND")


if __name__ == "__main__":

    darias = Darias()
    pick_trajectories = []
    place_trajectories = []
    pull_arm = None

    observer = DariasObserver(darias)

    darias.go_to_1(GoToTrajectory(**Home_Position), "RIGHT_ARM")
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


    darias.go_to_1(GoToTrajectory(**Home_Position), "RIGHT_ARM")
    n_boxes = 4
    for pick, place in zip(pick_trajectories, place_trajectories):
        open_hand()
        print("go over box")
        darias.go_to_1(pick, "RIGHT_ARM")
        grasp()
        print("build tower")
        darias.go_to_1(place, "RIGHT_ARM")

    open_hand()
    darias.go_to_1(pull_arm, "RIGHT_ARM")
    darias.kinesthetic("RIGHT_ARM")
    exit()
    #go_home()
    #open_hand()

    # open_hand = GoToTrajectory(duration=0.5, **observation)
    # open_hand.save("open_hand.npy")
    # exit()
    # darias.go_to_1(GoToTrajectory(duration=0.1, **Home_Position), "RIGHT_HAND")

    observer = DariasObserver(darias)
    observation = observer(*darias.groups["RIGHT_HAND"].refs)
    for _ in range(20):
        observation = move(observation, ['R_RIP', 'R_MIP', 'R_INP', 'R_SMP', 'R_THP'], displacement=0.05)
        darias.go_to_1(GoToTrajectory(duration=0.1, **observation), "RIGHT_HAND")
        raw_input("press a key")



    fin_traj = GoToTrajectory(duration=0.1, **observation)
    fin_traj.save("traj.npy")



