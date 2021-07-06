from dariaspy.franka_interface import Franka
from dariaspy.observers import RobotOserver
from dariaspy.recording import Recorder
from dariaspy.trajectory import GoToTrajectory, NamedTrajectory, LoadTrajectory
import matplotlib.pyplot as plt

import time

if __name__ == "__main__":
    franka = Franka()
    trajectory = LoadTrajectory("franka_trajectory.npy")

    dic = trajectory.get_dict_values()
    for k in dic.keys():
        plt.title(k)
        plt.plot(dic[k])
        plt.savefig("%s.png" % k)

    values = {k: trajectory.get_dict_values()[k][0] for k in trajectory.get_dict_values().keys()}
    print("first traj")
    init_trajectory = GoToTrajectory(3.0, **values)

    franka.go_to(init_trajectory, "arm")

    print("goto in 3 sec")
    time.sleep(3.)
    print("second traj")
    franka.go_to(trajectory, "arm")