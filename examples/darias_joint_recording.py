"""
Example
1. Set the robot in teaching mode
2. Record the trajectory (in RL the left arm will just drop down, on the real robot the user can move it in
gravity compensation)
3. The arm will be brought to its initial position
4. The recorded trajectory will be imitated.

Everything will happen in the cartesian space.
"""

from dariaspy.darias_interface import Darias
from dariaspy.positions import Home_Position
from dariaspy.recording import Recorder
from dariaspy.trajectory import GoToTrajectory
from dariaspy.observers import DariasObserver

if __name__ == "__main__":
    darias = Darias()

    observer = DariasObserver(darias)

    print("Go.")
    darias.go_to(GoToTrajectory(duration=5., **Home_Position), "RIGHT_ARM")

    print("Arm in Home Position.")
    darias.kinesthetic("RIGHT_ARM")
    print("Kinesthetic teaching")

    recording = Recorder(observer, observer.get_possible_refs(), sampling_frequency=10)

    print("Start recording")
    recording.record_fixed_duration(10.)
    print("Stop recording")

    darias.go_to(GoToTrajectory(duration=10., **Home_Position), "RIGHT_ARM")
    darias.go_to(recording.trajectory, "RIGHT_ARM")
