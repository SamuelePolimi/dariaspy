"""
Example - Bring the robot in its home position
"""

from dariaspy.franka_interface import Franka
from dariaspy.observers import RobotOserver
from dariaspy.recording import Recorder
import time

if __name__ == "__main__":

    franka = Franka()
    observer = RobotOserver(franka)
    values = observer(*observer.get_possible_refs())

    recording = Recorder(observer, observer.get_possible_refs(), sampling_frequency=20)

    for i in range(5):
        print(i)
        time.sleep(1)

    print("Start recording")
    recording.record_fixed_duration(10.)
    print("End recording")

    recording.trajectory.save("franka_trajectory.npy")
    print("file saved")

