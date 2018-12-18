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
        """
        Record a trajectory of fixed time-length.
        :param duration: Duration of the trajectory expressed in seconds.
        :type duration: float
        :return:
        """

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

    def conditional_record(self, callback_start, callback_end, max_duration=60.):
        """
        This is a conditional recording. the recording will start as soon as callback_start will return true.
        :param callback_start: Function which receives a goal as input, and produce a decision. When the decision is true,
        then the recording starts
        :param callback_end: Function which receives a goal as input, and produce a decision.
        When the decision is true, then the record will stop, and this method will return.
        :param max_duration: the overall process cannot exceed the max_duration, expressed in seconds.
        :return:
        """
        duration = 0.
        self.robot.mode.set_mode(DariasKinestheticMode)

        while not callback_start(self._record_goal()) and duration < max_duration:
            time.sleep(self.dt)
            duration += self.dt

        goal = self._record_goal()
        while not callback_end(goal) and duration < max_duration:
            self.trajectory.goal_list.append(goal)
            goal = self._record_goal()
            time.sleep(self.dt)
            duration += self.dt

        if duration >= max_duration:
            print("Max duration reached")


class WaitingToStart:

    def __init__(self, threshold=0.01, verbose=True):
        """
        This creates a callback for conditional_recording. The recording will start only when the mean velocity of each
        joint or position is greater then the threshold.
        :param threshold: minimum mean velocity for the recording to start
        :type threshold: float
        :param verbose: prompt on the console the activation of the recording
        :type verbose: bool
        """
        self.threshold = threshold
        self.current_position = None
        self.verbose = verbose

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
            print((self.current_position - goal.position).mean()/goal.duration)
            self.current_position = goal.position
            return False
        if self.verbose:
            print("Recording started %s" % str(time.time()))
        return True


class WaitingToEnd:

    def __init__(self, threshold=0.01, duration=1., verbose=True):
        """

        :param threshold: max velocity in order to consider the arm not moving
        :param duration: resting time in seconds
        """
        self.threshold = threshold
        self.current_position = None
        self.max_duration = duration
        self.duration = 0.
        self.verbose = verbose

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
                if self.verbose:
                    print("Recording stopped %s" % str(time.time()))
                return True
        else:
            self.current_position = goal.position
            self.duration = 0.
        return False
