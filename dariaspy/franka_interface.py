#!/usr/bin/env python
"""
This module contains the interface for Darias.
Darias is viewed as an object with a set of joints and two end-effectors.

The end-effector (left and right) expose their positions in 3D coordinates (with relative orientations).
The joints can be divided in two logical groups (left and right), and they expose the angle in radiants.
In future the joints will be divided in four groups (left-arm; left-hand; right-arm; right-hand).

Darias interfaces exposes also two methods, one for control and one for kinesthetic teaching.
"""
import rospy
import numpy as np
import time
import actionlib
import enum
import scipy
from scipy.interpolate import CubicSpline

from dariaspy.ros_listener import RosListener
from dariaspy.groups import Group, group_union
from dariaspy.trajectory import NamedTrajectoryBase
from dariaspy.movement_primitives import MovementPrimitive

from franka_core_msgs.msg import JointCommand
from franka_core_msgs.msg import RobotState, EndPointState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from franka_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from franka_msgs.msg import ErrorRecoveryActionGoal
from franka_tools import FrankaFramesInterface, FrankaControllerManagerInterface, JointTrajectoryActionClient, CollisionBehaviourInterface

from franka_interface import ArmInterface, RobotEnable, GripperInterface

import franka_dataflow

@enum.unique
class FrankaMode(enum.IntEnum):
    """
        Enum class for specifying and retrieving the current robot mode.
    """
    # ----- access using parameters name or value
    # ----- eg. RobotMode(0).name & RobotMode(0).value
    # ----- or  RobotMode['ROBOT_MODE_OTHER'].name & RobotMode['ROBOT_MODE_OTHER'].value

    ROBOT_MODE_OTHER = 0
    ROBOT_MODE_IDLE = 1
    ROBOT_MODE_MOVE = 2
    ROBOT_MODE_GUIDING = 3
    ROBOT_MODE_REFLEX = 4
    ROBOT_MODE_USER_STOPPED = 5
    ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY = 6


class Joint:
    """
    Defines a robotic joint.
    """

    def __init__(self, position, velocity):
        self.position = position
        self.velocity = velocity


class GeometricPoint(RosListener):

    def __init__(self):
        RosListener.__init__(self)
        self.translation = np.array([0., 0., 0.])
        self.rotation = np.array([0., 0., 0., 0.])

    def _callback(self, data):
        self.translation = np.array([
            data.transform.translation.x,
            data.transform.translation.y,
            data.transform.translation.z
        ])
        self.rotation = np.array([
            data.transform.rotation.w,
            data.transform.rotation.x,
            data.transform.rotation.y,
            data.transform.rotation.z
        ])

    @property
    def position(self):
        return np.concatenate([self.translation, self.rotation], axis=0)


class EndEffector(GeometricPoint):

    def __init__(self, left=True):
        GeometricPoint.__init__(self)
        rospy.Subscriber('/darias_control/darias/ENDEFF_%s_ARM_pos' % ('LEFT' if left else 'RIGHT'), TransformStamped,
                         self.get_callback())


class Arms(RosListener):
    """
    Defines the robotics' arms.
    We can access to the left arm or to the right one.
    """

    def __init__(self, robot):
        RosListener.__init__(self)
        # TODO: this will not work with gazebo.
        rospy.Subscriber('/joint_states', JointState,
                         self.get_callback())
        # rospy.Subscriber(robot._root + '/custom_franka_state_controller/joint_states', JointState,
        #                  self.get_callback())
        self.right = Joint(None, None)
        self.left = Joint(None, None)
        self.order = None
        self.info = {}
        self.update_fr = 0.
        self._last_time = time.time()

    def _callback(self, data):
        for i, ref in enumerate(data.name):
            self.info[ref] = data.position[i]

        if self.order is None:
            self.order = data.name


class Franka:

    def __init__(self, local=True):
        """
        Instantiates an interface for darias.
        """
        # if not local
        self._enabled = True
        self._root = "/franka_ros_interface"
        # if local
        # self._root = "/panda_simulator"
        # rospy.init_node("franka_robot_gym")
        rospy.init_node("robopy_interface")
        self.arms = Arms(self)
        self._arm_interface = ArmInterface(True)
        self._gripper_interface = GripperInterface()
        self._robot_status = RobotEnable()
        self._ctrl_manager = FrankaControllerManagerInterface(
            ns=self._root, sim=False)    # TODO: (sim=false for real robot))
        print(self._ctrl_manager)
        self._movegroup_interface = None    # TODO: consider removing it

        self._joint_command_publisher = rospy.Publisher(
            self._root + '/motion_controller/arm/joint_commands',
            # 'joint_commands',
            JointCommand,
            tcp_nodelay=True,
            queue_size=1)

        while not(self.arms.ready):
            pass

        self.groups = {}
        self._building_groups()

        # TODO: temporary
        self._joint_names = self.groups["arm_gripper"].refs

    def joint_names(self):
        return self._joint_names

    def _building_groups(self):
        self.groups["arm"] = Group("arm", ["panda_joint%d" % i for i in range(1, 8)])
        self.groups["gripper"] = Group("arm", ["panda_finger_joint%d" % i for i in [1, 2]])
        self.groups["arm_gripper"] = group_union("arm_gripper", self.groups["arm"], self.groups["gripper"])

    def _get_cubic_spline(self, trajectory, group_name, frequency=20):
        x = []
        y = []
        ts = []
        x_tot = 0.

        dt = 1 / frequency

        for goal, d in trajectory:
            x_tot += d
            ts.append(x_tot)
            y.append(np.array([goal[k] for k in self.groups[group_name].refs]))
            x.append(x_tot)
        x = np.array(x)
        y = np.array(y)
        print(x, y)
        spline = CubicSpline(x, y)

        points = int(frequency * x_tot)

        positions = []
        velocities = []
        timings = []
        for i in range(points):
            t = i*dt
            positions.append(spline(t))
            velocities.append(spline(t, 1))
            timings.append(t)
        return positions, velocities, timings

    def go_to(self, trajectory: NamedTrajectoryBase, group_name="arm_gripper", frequency=50):
        """
        (Blocking) Commands the limb to the provided positions.
        Waits until the reported joint state matches that specified.
        This function uses a low-pass filter using JointTrajectoryService
        to smooth the movement or optionally uses MoveIt! to plan and
        execute a trajectory.

        :type positions: dict({str:float})
        :param positions: joint_name:angle command
        :type timeout: float
        :param timeout: seconds to wait for move to finish [15]
        :type threshold: float
        :param threshold: position threshold in radians across each joint when
         move is considered successful [0.00085]
        :param test: optional function returning True if motion must be aborted
        :type use_moveit: bool
        :param use_moveit: if set to True, and movegroup interface is available,
         move to the joint positions using moveit planner.
        """

        curr_controller = self._ctrl_manager.set_motion_controller(
            self._ctrl_manager.joint_trajectory_controller)

        # traj_client = JointTrajectoryActionClient(
        #     joint_names=self.joint_names())
        traj_client = JointTrajectoryActionClient(
            joint_names=self.groups[group_name].refs
        )
        traj_client.clear()

        if len(trajectory.duration) > 1:
            self._get_cubic_spline(trajectory, group_name)

            for i, (position, velocity, d) \
                    in enumerate(zip(*self._get_cubic_spline(trajectory, group_name, frequency=frequency))):

                traj_client.add_point(
                    positions=position.tolist(), time=float(d),
                    velocities=velocity.tolist())
        else:
            for goal, d in trajectory:
                position = [float(goal[k]) for k in self.groups[group_name].refs]
                velocity = [0.]*len(position)
                print("position", position)
                print("velocity", velocity)
                print("d", float(d))
                traj_client.add_point(
                    positions=position, time=float(d),
                    velocities=velocity)

        traj_client.start()  # send the trajectory action request
        # traj_client.wait(timeout = timeout)

        franka_dataflow.wait_for(
            test=lambda: traj_client.result(),
            timeout=260,
            timeout_msg="Something did not work",
            rate=100,
            raise_on_error=False
        )
        res = traj_client.result()
        if res is not None and res.error_code:
            rospy.loginfo("Trajectory Server Message: {}".format(res))

        rospy.sleep(0.5)

        self._ctrl_manager.set_motion_controller(curr_controller)

        rospy.loginfo("{}: Trajectory controlling complete".format(
            self.__class__.__name__))
        rospy.sleep(0.5)

    def goto_mp(self, movement_primitive: MovementPrimitive, frequency=50, duration=10., group_name="arm_gripper"):

        curr_controller = self._ctrl_manager.set_motion_controller(
            self._ctrl_manager.joint_trajectory_controller)

        traj_client = JointTrajectoryActionClient(
            joint_names=self.groups[group_name].refs)
        traj_client.clear()

        pos_traj = movement_primitive.get_full_trajectory(frequency=frequency, duration=duration)
        vel_traj = movement_primitive.get_full_trajectory_derivative(frequency=frequency, duration=duration)

        timing = []
        positions = []
        velocities = []

        d_tot = 0
        for goal, d in pos_traj:
            d_tot += float(d)
            timing.append(d_tot)
            positions.append([goal[k] for k in self.groups[group_name].refs])

        for goal, d in vel_traj:
            velocities.append([goal[k]/duration for k in self.groups[group_name].refs])

        for t, position, velocity in zip(timing, positions, velocities):

            traj_client.add_point(
                    positions=position, time=t,
                    velocities=velocity)

        print("COMMAND SENT")
        traj_client.start()  # send the trajectory action request
        # traj_client.wait(timeout = timeout)

        franka_dataflow.wait_for(
            test=lambda: traj_client.result(),
            timeout=60,
            timeout_msg="Something did not work",
            rate=100,
            raise_on_error=False
        )
        res = traj_client.result()
        if res is not None and res.error_code:
            rospy.loginfo("Trajectory Server Message: {}".format(res))

        rospy.sleep(0.5)

        self._ctrl_manager.set_motion_controller(curr_controller)

        rospy.loginfo("{}: Trajectory controlling complete".format(
            self.__class__.__name__))
        rospy.sleep(0.5)
