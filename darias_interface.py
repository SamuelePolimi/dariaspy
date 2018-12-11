#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from ias_robot_msgs.msg import GoToGoal, State, GoToAction
import actionlib
import time
import numpy as np


class RosListener:

    def __init__(self):
        pass

    def _callback(self, data):
        raise NotImplementedError()

    def get_callback(self):
        return lambda x: self._callback(x)


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


class Joint(RosListener):

    def __init__(self):
        RosListener.__init__(self)
        rospy.Subscriber('/darias_control/darias/joint_state', JointState,
                         self.get_callback())
        self.name = None
        self.position = None
        self.velocity = None

    def _callback(self, data):
        if self.name is None:
            self.name = data.name
        self.position = np.array(data.position)
        self.velocity = np.array(data.velocity)


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
        self.joint = Joint()

        #########################################
        # Action
        #########################################
        self._handle_goto_service = actionlib.SimpleActionClient('/darias_control/darias/goto', GoToAction)
        self._handle_goto_service.wait_for_server()

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


arm = Darias()
time.sleep(1)
goal = np.random.standard_normal(6)
print(goal)
arm.go_to_joint(goal*0., 10., True)
time.sleep(10.)
