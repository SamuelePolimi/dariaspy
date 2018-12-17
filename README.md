DariasPy
========

A python interface for Darias!

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
- [ ] Record Trajectory 
- [ ] Include Optitrack