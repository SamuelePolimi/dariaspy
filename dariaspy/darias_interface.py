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
from ias_robot_msgs.srv import SettingsUpdate, SettingsUpdateRequest, KinestheticRequest, Kinesthetic, RobotInformation, RobotInformationRequest

import actionlib
import numpy as np
from enum import Enum

from ros_listener import RosListener, activate_listener
from trajectory import NamedTrajectoryBase
from groups import Group


class DariasMode(Enum):

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
            data.transform.rotation.x,
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
        :type mode: Enum

        :Example:

        >>> mode = DariasModeController()
        >>> mode.set_mode(DariasMode.DariasCommandMode)
        """
        settings_request = SettingsUpdateRequest()
        settings_request.mask = settings_request.MODE
        settings_request.settings.mode = mode.value
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


class InfoExposer:

    def __init__(self, *info_esposers):
        """
        An info exposer object exposes the information about some quantities.
        It can also 
        :param info_esposers:
        """


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
        self.info = {}

    def _callback(self, data):
        for i, ref in enumerate(data.name):
            self.info[ref] = data.position[i]

        if self.order is None:
            self.order = data.name[15:22] + data.name[:15] + data.name[37:44] + data.name[22:37]
        #TODO: old code!
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

        self.groups = {}
        self._building_groups()


        while not(self.arms.ready and self.left_end_effector.ready and self.right_end_effector.ready):
            pass

    def go_to(self, trajectory, group_name, goal_weight=1.0):
        """
        It performs a desired trajectory.

        :param trajectory: trajectory to follow
        :type trajectory: NamedTrajectoryBase
        :param group_name: Name of the group
        :type group_name: str
        """
        self.mode.set_mode(DariasMode.DariasCommandMode)

        # Construct the Command Message:
        joint_goal = GoToGoal()
        if group_name in ['ENDEFF_LEFT_ARM', 'ENDEFF_RIGHT_ARM']:
            joint_goal.type = joint_goal.CART
        else:
            joint_goal.type = joint_goal.JOINT

        joint_goal.group = self.groups[group_name].group_name

        joint_goal.weight = goal_weight
        joint_goal.priority = joint_goal.APPEND

        for goal, d in trajectory:
            joint_state = State()
            joint_state.duration = d
            joint_state.destination = [goal[k] for k in self.groups[group_name].refs]
            joint_goal.states.append(joint_state)

        self._handle_goto_service.send_goal(joint_goal)
        if True:
            self._handle_goto_service.wait_for_result()

    def kinesthetic(self, group):
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
        start_msg.teached_group = self.groups[group].group_name

        try:
            kin_service(start_msg)
        except rospy.ServiceException as exc:
            print("Kinesthetic Service error: " + str(exc))
            return

    def _building_groups(self):

        rospy.wait_for_service('/darias_control/darias/information')
        service = rospy.ServiceProxy('/darias_control/darias/information', RobotInformation)
        req = RobotInformationRequest()
        try:
            answer = service(req)
            for group in answer.joint_space_control:
                self.groups[group.name] = Group(group.name, group.joints)
            self.groups["ENDEFF_LEFT_ARM"] = Group("LEFT_ARM",
                                                   ["L_TX", "L_TY", "L_TZ", "L_RX", "L_RY", "L_RZ", "L_RW"])
            self.groups["ENDEFF_RIGHT_ARM"] = Group("RIGHT_ARM",
                                                    ["R_TX", "R_TY", "R_TZ", "R_RX", "R_RY", "R_RZ", "R_RW"])

        except rospy.ServiceException as exc:
            print("Information Service error: " + str(exc))
            return

