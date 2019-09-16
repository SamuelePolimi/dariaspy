"""
Example - Bring the robot in its home position
"""

from dariaspy.ros_listener import activate_listener
from dariaspy.darias_interface import Darias
from dariaspy.positions import Home_Position
from dariaspy.trajectory import GoToTrajectory

if __name__ == "__main__":

    activate_listener()

    darias = Darias()
    print("Home position")
    darias.go_to(GoToTrajectory(**Home_Position), "RIGHT_ARM")

