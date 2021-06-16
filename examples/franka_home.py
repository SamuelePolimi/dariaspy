"""
Example - Bring the robot in its home position
"""

from dariaspy.franka_interface import Franka
from dariaspy.observers import RobotOserver
import time

if __name__ == "__main__":

    franka = Franka()
    observer = RobotOserver(franka)
    values = observer(*observer.get_possible_refs())
    for k in values.keys():
        values[k] = 0.
    values['panda_joint7'] -= 0.15
    print(values)
    for i in range(20):
        franka.set_joint_velocities(values)
        time.sleep(0.05)
    # print(observer.get_possible_refs())
    # print(observer(*observer.get_possible_refs()))
    # print(franka.arms.info)
    # print("Home position")
    # darias.go_to(GoToTrajectory(**Home_Position), "RIGHT_ARM")

