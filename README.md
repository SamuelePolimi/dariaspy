DariasPy
========

> **WARNING**: The new updates requires **robcom interface for python** (recompile robcom allowing in the configuration python).

> **WARNING**: From now on it is necessary to import **activate listener** from dariaspy.ros_listener and execute it before anything else.


A python interface for Darias!

With this library it is possible to perform simple robotic experiments with a minimal effort. 
For a complete documentation look into [DariasPy Guide](https://samuelepolimi.github.io/DariasPy-Doc/)
For example, for running darias in its home position, it is sufficient to write:

```python
from dariaspy.ros_listener import activate_listener
from dariaspy.darias_interface import Darias
from dariaspy.positions import Home_Position
from dariaspy.trajectory import GoToTrajectory

if __name__ == "__main__":

    activate_listener()

    darias = Darias()
    print("Home position")
    darias.go_to(GoToTrajectory(**Home_Position), "RIGHT_ARM")
```

or, for recording a trajectory and repeating it, is sufficient to write:

```python

from dariaspy.ros_listener import activate_listener
from dariaspy.darias_interface import Darias
from dariaspy.positions import Home_Position
from dariaspy.recording import Recorder
from dariaspy.trajectory import GoToTrajectory
from dariaspy.observers import DariasObserver

if __name__ == "__main__":
    
    activate_listener()
    
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
```

The library already exposes some nice features, like the possibility to starting the trajectory conditioning on a specific
event (such as velocity exceeding a threshold), but it will be extended in the near future with more specific funtionalities.

A long-term goal would ideally be to have a simpler and common interface to all our robots :).

References to Darias
--------------------

How to install the needed software (Ros, IASRos, Robcom) [Guide by Koert](https://git.ias.informatik.tu-darmstadt.de/ias_ros/ias_ros_core).
How to start Darias [Guide by Stark and Koert](https://git.ias.informatik.tu-darmstadt.de/ausy/wiki/blob/master/tutorial_darias_right_arm/Using_DARIAS_Right_Arm2.pdf).
How to mount the DLR hand and use it [DLR Hands Guide](dariashand.pdf).

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
- [x] Transormation relative to a reference-frame
- [x] Create `observers`. Will be useful either for distinguish arm and hand, an also for generalizing recording.
- [x] Record extended/generalized to optitrack objects
- [x] Hand control
- [x] ProMPs execution
- [ ] Move to Robcom2 once it offers promps
- [ ] General interface with other robots
- [ ] Better guide, finer documentation
- [ ] Exposition of shell commands
- [ ] Gui interface for status and commands