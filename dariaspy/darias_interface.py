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
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from ias_robot_msgs.msg import GoToGoal, State, GoToAction
from ias_robot_msgs.srv import SettingsUpdate, SettingsUpdateRequest, KinestheticRequest, Kinesthetic
import actionlib
import numpy as np

from ros_listener import RosListener, activate_listener
from darias_space import Trajectory, TrajectoryType


class DariasMode:

    DariasCommandMode = 3
    DariasKinestheticMode = 4


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
            data.transform.rotation.z,
            data.transform.rotation.y,
            data.transform.rotation.z
        ])

    @property
    def position(self):
        return np.concatenate([self.translation, self.rotation], axis=0)


class DariasModeController:

    def __init__(self):
        self.settings_service = rospy.ServiceProxy('/darias_control/darias/settings', SettingsUpdate)
        self.set_mode(DariasMode.DariasCommandMode)

    def set_mode(self, mode):
        """
        It sets a different mode.

        :param mode: DariasMode.<mode>
        :type mode: int

        :Example:

        >>> mode = DariasModeController()
        >>> mode.set_mode(DariasMode.DariasCommandMode)
        """
        settings_request = SettingsUpdateRequest()
        settings_request.mask = settings_request.MODE
        settings_request.settings.mode = mode
        try:
            self.settings_service(settings_request)
        except rospy.ServiceException as exc:
            print("Settings Service error: " + str(exc))
            return
        self.mode = mode


class Joint:
    """
    Defines a robotic joint.
    """

    def __init__(self, position, velocity):
        self.position = position
        self.velocity = velocity


class Hand:

    def __init__(self):
        self.fingers = [np.zeros(3) for _ in range(5)]

    def update(self, info):
        for i in range(5):
            self.fingers[i] = info[0+i*3:(i+1)*3]


class Arms(RosListener):
    """
    Defines the robotics' arms.
    We can access to the left arm or to the right one.
    """

    def __init__(self):
        RosListener.__init__(self)
        rospy.Subscriber('/darias_control/darias/joint_state', JointState,
                         self.get_callback())
        self.right = Joint(None, None)
        self.left = Joint(None, None)
        self.order = None

    def _callback(self, data):

        if self.order is None:
            self.order = data.name[15:22] + data.name[:15] + data.name[37:44] + data.name[22:37]
        self.right.position = np.concatenate([data.position[15:22], data.position[:15]], axis=0)
        self.left.position = np.concatenate([data.position[37:44], data.position[22:37]], axis=0)
        self.right.velocity = np.concatenate([data.velocity[15:22], data.velocity[:15]], axis=0)
        self.left.velocity = np.concatenate([data.velocity[37:44], data.velocity[22:37]], axis=0)


class EndEffector(GeometricPoint):

    def __init__(self, left=True):
        GeometricPoint.__init__(self)
        rospy.Subscriber('/darias_control/darias/ENDEFF_%s_ARM_pos' % ('LEFT' if left else 'RIGHT'), TransformStamped,
                         self.get_callback())


class Darias:

    def __init__(self):
        """
        Instantiates an interface for darias.
        """

        print("before activating")
        activate_listener()

        #########################################
        # State
        #########################################
        self.left_end_effector = EndEffector(True)
        self.right_end_effector = EndEffector(False)
        self.arms = Arms()
        self.mode = DariasModeController()

        #########################################
        # Action
        #########################################
        self._handle_goto_service = actionlib.SimpleActionClient('/darias_control/darias/goto', GoToAction)
        self._handle_goto_service.wait_for_server()

        while not(self.arms.ready and self.left_end_effector.ready and self.right_end_effector.ready):
            pass

    def go_to(self, trajectory, left=True, wait=True):
        """
        It performs a desired trajectory.

        :param trajectory: trajectory to follow
        :type trajectory: Trajectory
        :param left: left or right arms
        :type left: bool
        :param wait: wait for the trajectory to be done
        :type wait: bool
        """
        self.mode.set_mode(3)

        # Construct the Command Message:
        joint_goal = GoToGoal()
        trajectory_type = trajectory.get_type()
        if trajectory_type == TrajectoryType.CartTrajectoryType:
            joint_goal.type = joint_goal.CART
        elif trajectory_type == TrajectoryType.JointTrajectoryType:
            joint_goal.type = joint_goal.JOINT
        else:
            raise Exception("The trajectory cannot contain goal of different typology (MixedTrajectoryType)")

        joint_goal.group = "%s_ARM" % ("LEFT" if left else "RIGHT")

        for goal in trajectory.goal_list:
            joint_state = State()
            joint_state.duration = goal.duration
            joint_state.destination = goal.position.tolist()
            joint_goal.states.append(joint_state)

        self._handle_goto_service.send_goal(joint_goal)
        if wait:
            self._handle_goto_service.wait_for_result()

    def kinesthetic(self, left=True):
        """
        This method provide the kinerthetic teaching (i.e., gravity compensation mode).
        In order to stop this service it is sufficient to call mode.set_mode(DariasCommandMode)

        :param left: left or right arm?
        :type left: bool
        """

        self.mode.set_mode(DariasMode.DariasKinestheticMode)

        rospy.wait_for_service('/darias_control/darias/kinesthetic')
        kin_service = rospy.ServiceProxy('darias_control/darias/kinesthetic', Kinesthetic)
        start_msg = KinestheticRequest()
        start_msg.mirrored = False
        if left:
            start_msg.teached_group = 'LEFT_ARM'
        else:
            start_msg.teached_group = 'RIGHT_ARM'

        try:
            kin_service(start_msg)
        except rospy.ServiceException as exc:
            print("Kinesthetic Service error: " + str(exc))
            return

