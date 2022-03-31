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

if __name__ == "__main__":

    franka = Franka()

    trajectory_1 = LoadTrajectory("franka_trajectory_1.npy")
    trajectory_2 = LoadTrajectory("franka_trajectory_2.npy")

    learning_group = "arm"

    franka._gripper_interface.open()
    ms = ClassicSpace(franka.groups[learning_group], n_features=30)

    # Learn te movement in this space
    mp1 = LearnTrajectory(ms, trajectory_1)
    print("Go to the initial point of the movement primitive")
    tr_init = mp1.get_init_trajectory(10.)

    # Go to the correct position
    franka.go_to(tr_init, learning_group)

    print("Play the movement primitive")
    # Go to with movement primitive

    franka.goto_mp(mp1, frequency=5, group_name=learning_group)
    franka._gripper_interface.close()

    mp2 = LearnTrajectory(ms, trajectory_2)
    print("Go to the initial point of the movement primitive")
    tr_init = mp2.get_init_trajectory(1.)

    franka.go_to(tr_init, learning_group)

    franka.goto_mp(mp2, frequency=5, group_name=learning_group)
    franka._gripper_interface.open()

    franka._gripper_interface.stop_action()

