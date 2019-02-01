"""
Example - Bring the robot in its home position
"""

from dariaspy.darias_interface import Darias
from dariaspy.positions import Home_Right_Joints, Home_Left_Joints, Home_Position
from dariaspy.darias_space import Trajectory, JointGoal
from dariaspy.observers import DariasObserver
from dariaspy.recording import GoToTrajectory

if __name__ == "__main__":

    darias = Darias()

    print("Home position")
    darias.go_to_1(GoToTrajectory(**Home_Position), "WHOLE_ROBOT")

