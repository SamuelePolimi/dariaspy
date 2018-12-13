import numpy as np
from core.darias_interface import Darias
from core.positions import Home_Right_Joints, Home_Left_Joints

darias = Darias()

darias.go_to_joint(np.array(Home_Left_Joints), 10., True)
darias.go_to_joint(np.array(Home_Right_Joints), 10., False)