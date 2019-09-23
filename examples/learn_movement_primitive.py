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

from dariaspy.ros_listener import activate_listener
from dariaspy.darias_interface import Darias
from dariaspy.positions import Home_Position
from dariaspy.recording import Recorder
from dariaspy.trajectory import GoToTrajectory
from dariaspy.observers import DariasObserver, EndEffectorObserver, JointObserver
from dariaspy.movement_primitives import LearnTrajectory, ClassicSpace

if __name__ == "__main__":

    activate_listener()

    darias = Darias()


    # JointGroup allow to work in both joint space and task space.
    observer = JointObserver(DariasObserver(darias), EndEffectorObserver(darias))

    print("Go.")
    darias.go_to(GoToTrajectory(duration=5., **Home_Position), "RIGHT_ARM")

    print("Arm in Home Position.")
    darias.kinesthetic("RIGHT_ARM")
    print("Kinesthetic teaching")

    recording = Recorder(observer, observer.get_possible_refs(), sampling_frequency=10)

    print("Start recording")
    recording.record_fixed_duration(10.)
    print("Stop recording")

    # You can change to "ENDEFF_RIGHT_ARM" if you want to work in task_space
    learning_group = "RIGHT_ARM"

    # Define the functional space of the movements
    ms = ClassicSpace(darias.groups[learning_group],n_features=10)
    # Learn te movement in this space
    mp = LearnTrajectory(ms, recording.trajectory)

    print("Go to the initial point of the movement primitive")
    tr_init = mp.get_init_trajectory(10.)

    # Go to the correct position
    darias.go_to(tr_init, learning_group)

    print("Play the movement primitive")
    # Go to with movement primitive
    darias.goto_mp(mp)
