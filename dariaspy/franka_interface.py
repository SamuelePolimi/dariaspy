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

from dariaspy.ros_listener import RosListener
from franka_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from franka_msgs.msg import ErrorRecoveryActionGoal
from franka_core_msgs.msg import RobotState

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
        rospy.Subscriber(robot._root + '/custom_franka_state_controller/joint_states', JointState,
                         self.get_callback())
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
            # todo check this one
            self.order = data.name[15:22] + data.name[:15] + data.name[37:44] + data.name[22:37]


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
        rospy.init_node("franka_robot_gym")
        self.arms = Arms(self)

        self._joint_command_publisher = rospy.Publisher(
            self._root + '/motion_controller/arm/joint_commands',
            JointCommand,
            tcp_nodelay=True)

        while not(self.arms.ready):
            pass

    def set_joint_positions(self, positions):
        """
        Commands the joints of this limb to the specified positions.

        :type positions: dict({str:float}
        :param positions: dict of {'joint_name':joint_position,}
        """
        command_msg = JointCommand()
        joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6',
                       'panda_joint7']
        command_msg.names = joint_names
        command_msg.position = [positions[j] for j in joint_names]
        command_msg.mode = JointCommand.POSITION_MODE
        command_msg.header.stamp = rospy.Time.now()
        self._joint_command_publisher.publish(command_msg)

    def set_joint_velocities(self, velocities):
        """
        Commands the joints of this limb to the specified velocities.

        :type velocities: dict({str:float})
        :param velocities: dict of {'joint_name':joint_velocity,}
        """
        # self._toggle_enabled(True)
        command_msg = JointCommand()
        joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6',
                       'panda_joint7']
        command_msg.names = joint_names
        command_msg.velocity = [velocities[j] for j in joint_names]
        command_msg.mode = JointCommand.VELOCITY_MODE
        command_msg.header.stamp = rospy.Time.now()
        print(command_msg)
        self._joint_command_publisher.publish(command_msg)

    def _toggle_enabled(self, status):

        pub = rospy.Publisher('{}/franka_control/error_recovery/goal'.format(self._root), ErrorRecoveryActionGoal,
                              queue_size=10)

        if self._enabled == status:
            print("already")
            #rospy.loginfo("Robot is already %s" % self.state())

        franka_dataflow.wait_for(
            test=lambda: self._enabled == status,
            timeout=5.0,
            timeout_msg=("Failed to %sable robot" %
                         ('en' if status else 'dis',)),
            body=lambda: pub.publish(ErrorRecoveryActionGoal()),
        )
        rospy.loginfo("Robot %s", ('Enabled' if status else 'Disabled'))
