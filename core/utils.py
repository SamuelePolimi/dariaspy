from darias_interface import Darias, DariasKinestheticMode
from darias_space import Trajectory, JointGoal, CartGoal, Goal
import time
import numpy as np

CartRecordMode = 0
JointRecordMode = 1


class Record:

    def __init__(self, robot, record_mode=0, left=True, sampling_frequency=10):
        """

        :param robot:
        :type robot: Darias
        """
        self.robot = robot
        self.left = left
        self.mode = record_mode
        self.trajectory = Trajectory([])
        self.sampling_frequency = sampling_frequency
        self.dt = 1. / sampling_frequency

    def record_fixed_duration(self, duration=10.):

        # TODO: not sure it is working
        self.robot.mode.set_mode(DariasKinestheticMode)

        for t in range(int(duration * self.sampling_frequency)):
            self.trajectory.goal_list.append(self._record_goal())
            time.sleep(self.dt)

    def _record_goal(self):
        if self.mode == JointRecordMode:
            if self.left:
                return JointGoal(self.robot.arms.left.position, duration=self.dt)
            else:
                return JointGoal(self.robot.arms.right.position, duration=self.dt)
        elif self.mode == CartRecordMode:
            if self.left:
                return CartGoal(self.robot.left_end_effector.position, duration=self.dt)
            else:
                return CartGoal(self.robot.right_end_effector.position, duration=self.dt)

    def conditional_record(self, callback_start, callback_end):

        # TODO: not sure it is working
        self.robot.mode.set_mode(DariasKinestheticMode)

        while not callback_start(self._record_goal()):
            time.sleep(self.dt)

        goal = self._record_goal()
        while not callback_end(goal):
            self.trajectory.goal_list.append(goal)
            goal = self._record_goal()
            time.sleep(self.dt)


class WaitingToStart:

    def __init__(self, threshold=0.1):
        self.threshold = threshold
        self.current_position = None

    def __call__(self, goal):
        """

        :param goal:
        :type goal: Goal
        :return:
        :rtype: bool
        """
        if self.current_position is None:
            self.current_position = goal.position
            return False
        elif np.abs(self.current_position - goal.position).mean()/goal.duration < self.threshold:
            self.current_position = goal.position
            return False
        return True


class WaitingToEnd:

    def __init__(self, threshold=0.1, duration=1.):
        """

        :param threshold: max velocity in order to consider the arm not moving
        :param duration: resting time in seconds
        """
        self.threshold = threshold
        self.current_position = None
        self.max_duration = duration
        self.duration = 0.

    def __call__(self, goal):
        """

        :param goal:
        :type goal: Goal
        :return:
        :rtype: bool
        """
        if self.current_position is None:
            self.current_position = goal.position
            return False
        elif np.abs(self.current_position - goal.position).mean()/goal.duration < self.threshold:
            self.current_position = goal.position
            self.duration += goal.duration
            if self.duration > self.max_duration:
                return True
        else:
            self.current_position = goal.position
            self.duration = 0.
        return False
