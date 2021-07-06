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
import matplotlib.pyplot as plt


from dariaspy.franka_interface import Franka
from dariaspy.positions import Home_Position
from dariaspy.recording import Recorder
from dariaspy.trajectory import GoToTrajectory, LoadTrajectory
from dariaspy.observers import DariasObserver, EndEffectorObserver, JointObserver
from dariaspy.movement_primitives import LearnTrajectory, ClassicSpace

if __name__ == "__main__":

    franka = Franka()

    trajectory = LoadTrajectory("franka_trajectory.npy")

    learning_group = "arm"
    ms = ClassicSpace(franka.groups[learning_group], n_features=10)

    # Learn te movement in this space
    mp = LearnTrajectory(ms, trajectory)

    print("Go to the initial point of the movement primitive")
    tr_init = mp.get_init_trajectory(10.)

    # Go to the correct position
    # franka.go_to(tr_init, learning_group)

    print("Play the movement primitive")
    # Go to with movement primitive

    # tr_vel = mp.get_full_trajectory_derivative()
    # tr_pos = mp.get_full_trajectory()
    # plt.plot(tr_vel.values[:, 2], label="velocoty")
    # plt.plot(tr_pos.values[:, 2], label="position")
    # plt.legend(loc="best")
    # plt.savefig("velocities.png")

    franka.goto_mp(mp, frequency=5)
