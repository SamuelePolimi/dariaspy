from core.darias_interface import Darias
from core.positions import Home_Right_Joints, Home_Left_Joints
from core.darias_space import Trajectory, JointGoal

darias = Darias()

print("Left arm going in Home position")
darias.go_to(Trajectory([JointGoal(Home_Left_Joints, 10.)]), left=True, wait=True)

print("Right arm going in Home position")
darias.go_to(Trajectory([JointGoal(Home_Right_Joints, 10.)]), left=False, wait=True)