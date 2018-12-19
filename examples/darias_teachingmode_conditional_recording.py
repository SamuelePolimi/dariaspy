"""
Example
1. Set the robot in teaching mode
2.1. The recording starts only when the user start moving the robot
2.2. Record the trajectory (in RL the left arm will just drop down, on the real robot the user can move it in
gravity compensation)
2.3 The recording stops when the user is not moving the arm anymore
3. The arm will be brought to its initial position
4. The recorded trajectory will be imitated.

Everything will happen in the joint space.
"""


from core.darias_interface import Darias
from core.darias_space import Trajectory, JointGoal
from core.utils import Record, RecordMode, WaitingToStart, WaitingToEnd

if __name__ == "__main__":
    darias = Darias()

    darias.kinesthetic(left = True)
    recording = Record(darias, record_mode=RecordMode.JointRecordMode, left=True, sampling_frequency=50)

    print("Start recording")
    recording.conditional_record(WaitingToStart(), WaitingToEnd())
    print("Stop recording")

    print("Go to the initial point")
    start_trajectory = Trajectory([JointGoal(recording.trajectory.goal_list[0].position, 10.)])
    darias.go_to(start_trajectory, left=True)
    print("Initial position reached")

    print("Start the recorded trajectory")
    darias.go_to(recording.trajectory, left=True)
    print("Done :)")
