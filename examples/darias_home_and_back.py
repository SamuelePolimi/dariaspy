from core.darias_interface import Darias
from core.positions import Home_Right_Joints, Home_Left_Joints
from core.darias_space import Trajectory, JointGoal


darias = Darias()

while not darias.arms.ready:
    pass

old_left_joint =  darias.arms.left.position
old_right_joint = darias.arms.right.position

darias.go_to(Trajectory([JointGoal(Home_Left_Joints, 10.)]), left=True, wait=True)
darias.go_to(Trajectory([JointGoal(Home_Right_Joints, 10.)]), left=False, wait=True)

darias.go_to(Trajectory([JointGoal(old_left_joint, 10.)]), left=True, wait=True)
darias.go_to(Trajectory([JointGoal(old_right_joint, 10.)]), left=False, wait=True)