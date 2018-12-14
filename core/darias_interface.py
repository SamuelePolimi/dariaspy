#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from ias_robot_msgs.msg import GoToGoal, State, GoToAction
import actionlib
import numpy as np


class RosListener:

    def __init__(self):
        self.ready = False

    def _internal_callback(self, data):
        self._callback(data)
        self.ready = True

    def _callback(self, data):
        raise NotImplementedError()

    def get_callback(self):
        return lambda x: self._internal_callback(x)


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


class Joint:

    def __init__(self, position, velocity):
        self.position = position
        self.velocity = velocity


class Arms(RosListener):

    def __init__(self):
        RosListener.__init__(self)
        rospy.Subscriber('/darias_control/darias/joint_state', JointState,
                         self.get_callback())
        self.right = Joint(None, None)
        self.left = Joint(None, None)
        self.order = None

    def _callback(self, data):

        if self.order is None:
            self.order = data.name[15:22] + data.name[:15] + data.name[27:44] + data.name[22:37]
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
        rospy.init_node("listener", anonymous=True)

        #########################################
        # State
        #########################################
        self.left_end_effector = EndEffector(True)
        self.right_end_effector = EndEffector(False)
        self.joint = Arms()

        #########################################
        # Action
        #########################################
        self._handle_goto_service = actionlib.SimpleActionClient('/darias_control/darias/goto', GoToAction)
        self._handle_goto_service.wait_for_server()

        while not(self.joint.ready and self.left_end_effector.ready and self.right_end_effector.ready):
            pass

    def go_to_joint(self, q_des, duration, left=False):
        # Construct the Command Message:
        joint_goal = GoToGoal()
        joint_goal.type = joint_goal.JOINT
        joint_goal.group = "%s_ARM" % ("LEFT" if left else "RIGHT")

        joint_state = State()
        joint_state.duration = duration
        joint_state.destination = q_des.tolist()
        joint_goal.states.append(joint_state)

        self._handle_goto_service.send_goal(joint_goal)

