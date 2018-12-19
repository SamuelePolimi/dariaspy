from core.darias_interface import Darias
from core.darias_space import Trajectory, JointGoal
from core.utils import Record, JointRecordMode, WaitingToStart, WaitingToEnd

darias = Darias()

recording = Record(darias, record_mode=JointRecordMode, left=True, sampling_frequency=50)
darias.kinesthetic(left=True)

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
