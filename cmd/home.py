"""
Example - Bring the robot in its home position
"""

from dariaspy.darias_interface import Darias
from dariaspy.positions import Home_Right_Joints, Home_Left_Joints
from dariaspy.darias_space import Trajectory, JointGoal

if __name__ == "__main__":
    darias = Darias()

    print("Left arm going in Home position")
    darias.go_to(Trajectory([JointGoal(Home_Left_Joints, 10.)]), left=True, wait=True)

    print("Right arm going in Home position")
    darias.go_to(Trajectory([JointGoal(Home_Right_Joints, 10.)]), left=False, wait=True)