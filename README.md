DariasPy
========

A python interface for Darias!

With this library it is possible to perform simple robotic experiments with a minimal effort. 
For a complete documentation look into [DariasPy Guide](https://samuelepolimi.github.io/DariasPy-Doc/)
For example, for running darias in its home position, it is sufficient to write:

```python
from core.darias_interface import Darias
from core.positions import Home_Right_Joints, Home_Left_Joints
from core.darias_space import Trajectory, JointGoal

darias = Darias()

darias.go_to(Trajectory([JointGoal(Home_Left_Joints, 10.)]), left=True, wait=True)

darias.go_to(Trajectory([JointGoal(Home_Right_Joints, 10.)]), left=False, wait=True)
```

or, for recording a trajectory and repeating it, is sufficient to write:

```python

from core.darias_interface import Darias
from core.darias_space import Trajectory, JointGoal
from core.utils import Record, RecordMode


darias = Darias()

darias.kinesthetic(left = True)
recording = Record(darias, record_mode=RecordMode.JointRecordMode, left=True)

print("Start recording")
recording.record_fixed_duration(10.)
print("Stop recording")

print("Go to the initial point")
start_trajectory = Trajectory([JointGoal(recording.trajectory.goal_list[0].position, 10.)])
darias.go_to(start_trajectory, left=True)
print("Initial position reached")

print("Start repeating the recorded trajectory")
darias.go_to(recording.trajectory, left=True)
print("Done :)")
```

The library already exposes some nice features, like the possibility to starting the trajectory conditioning on a specific
event (such as velocity exceeding a threshold), but it will be extended in the near future with more specific funtionalities.

A long-term goal would ideally be to have a simpler and common interface to all our robots :).

References to Darias
--------------------

How to install the needed software (Ros, IASRos, Robcom) [Guide by Koert](https://git.ias.informatik.tu-darmstadt.de/ias_ros/ias_ros_core).
How to start Darias [Guide by Stark and Koert](https://git.ias.informatik.tu-darmstadt.de/ausy/wiki/blob/master/tutorial_darias_right_arm/Using_DARIAS_Right_Arm2.pdf).

Start ros with:
````bash
roslaunch robcom_launch darias.launch
````

Start sl with:
```bash
cd ~/sl/build/darias/
./xdarias
```


Goals
-----

- [x] Initialize the library
- [x] Perceive cartisian and joint current positions
- [x] GoTo with joint coordinates
- [x] GoTo with 3D coordinates
- [x] Wait GoTo for finish
- [x] Definition of Trajectory
- [x] Robot mode
- [x] Record Trajectory 
- [x] Points from optitrack
- [ ] Transormation relative to a reference-frame
- [ ] Create `observers`. Will be useful either for distinguish arm and hand, an also for generalizing recording.
- [ ] Record extended/generalized to optitrack objects
- [ ] Hand control
- [ ] Possible interface with other robots