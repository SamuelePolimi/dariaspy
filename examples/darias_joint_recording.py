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
from dariaspy.positions import Home_Left_Joints, Home_Right_Joints
from dariaspy.darias_space import Trajectory, JointGoal
from dariaspy.recording import Recorder
from dariaspy.observers import DariasObserver

if __name__ == "__main__":
    darias = Darias()

    observer = DariasObserver(darias)
    print(darias.groups.keys())

    darias.go_to(Trajectory([JointGoal(Home_Left_Joints, 10.)]), left=True, wait=True)
    darias.go_to(Trajectory([JointGoal(Home_Right_Joints, 10.)]), left=False, wait=True)

    darias.kinesthetic("WHOLE_ROBOT")

    recording = Recorder(observer, observer.get_possible_refs(), sampling_frequency=10)

    print("Start recording")
    recording.record_fixed_duration(10.)
    print("Stop recording")

    darias.go_to(Trajectory([JointGoal(Home_Left_Joints, 10.)]), left=True, wait=True)
    darias.go_to(Trajectory([JointGoal(Home_Right_Joints, 10.)]), left=False, wait=True)
    darias.go_to_1(recording.trajectory, "RIGHT_ARM_HAND")
