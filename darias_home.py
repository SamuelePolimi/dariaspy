import numpy as np
from core.darias_interface import Darias
from core.positions import Home_Right_Joints, Home_Left_Joints

darias = Darias()
home_position = darias.joint.position
name_position = darias.joint.name

print("LEFT",
      darias.left_end_effector.translation,
        darias.left_end_effector.rotation)
print("RIGHT",
      darias.right_end_effector.translation,
        darias.right_end_effector.rotation)
print "Joints", home_position, name_position

darias.go_to_joint(np.array(Home_Left_Joints), 10., True)
darias.go_to_joint(np.array(Home_Right_Joints), 10., False)