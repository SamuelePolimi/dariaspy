"""
This module contains the definitions of goal (which are points in 3D or Joint space with a duration in time),
and of trajectories which can be viewed as a set of goals.
"""
"""
Deprecated!
"""
import numpy as np
from enum import Enum


class GoalDimension(Enum):
    CartGoalDimensions = 7
    JointGoalDimensions = 22


class TrajectoryType(Enum):
    CartTrajectoryType = 0
    JointTrajectoryType = 1
    MixedTrajectoryType = 2

@DeprecationWarning
class Goal:
    """
    This is the generic interface of a goal: a given position and the time needed to achieve it.
    """

    def __init__(self, position, duration, dimensions=7):
        """
        Instantiate a goal.

        :param position: Position of the goal (in a given space)
        :type position: np.nd_array
        :param duration: Duration for reaching the goal
        :type duration: float
        :param dimensions: Number of dimensions of the metric space
        :type dimensions: int
        """
        self.position = position if position is not None else np.zeros(dimensions)
        self.duration = duration
        self.dimensions = dimensions

    def _check(self):
        assert len(self.position.shape) == 1 and self.position.shape[0] == self.dimensions, \
            "Position must be a 1-dimensional array of length %d." % self.dimensions
        assert np.isscalar(self.duration), "Duration must be a scalar"

    def from_array(self, array):
        """
        Give a "well shaped" array, this method reconstruct a goal.

        :param array: set the goal from an array
        :type array: np.nd_array


        :todo:

        This might have more sense as a factory class
        """
        if array.shape[0] == self.dimensions + 1:
            self.position = array[:self.dimensions]
            self.duration = array[-1]
        else:
            raise Exception("Array dimension should be %d, since the first %d are reserved for the position and the last one for the duration" % (self.dimensions + 1, self.dimensions))

    def to_array(self):
        """
        Return a "well shaped" array representing the goal.
        TODO> This might have more sense as a factory class

        :return:
        :rtype: array containing the information about the goal
        """
        return np.concatenate([self.position, [self.duration]], axis=0)


@DeprecationWarning
class CartGoal(Goal):
    """
    Goal in the cartesian space.
    """

    def __init__(self, position=None, duration=1):
        Goal.__init__(self, position, duration, GoalDimension.CartGoalDimensions.value)


@DeprecationWarning
class JointGoal(Goal):
    """
    Goal in the joint space
    """

    def __init__(self, position=None, duration=1):
        Goal.__init__(self, position, duration, GoalDimension.JointGoalDimensions.value)


@DeprecationWarning
class Trajectory:
    """
    A trajectory is a ordered list of goals. You can combine goal in different spaces.
    """

    def __init__(self, goal_list):
        """
        Create a trajectory from a list of goals.

        :param goal_list: Ordered sequence of goal, provided as a list
        :type goal_list: list
        """
        self.goal_list = goal_list

    def from_list_of_array(self, loa):
        """
        Given a list of "well shaped" arrays, it recovers the trajectory.

        :param loa: list of goals
        :type loa: list
        """
        self.goal_list = []
        for array in loa:
            if array.shape[0] == GoalDimension.CartGoalDimensions.value + 1:
                goal = CartGoal()
                goal.from_array(array)
                self.goal_list.append(goal)
            elif array.shape[0] == GoalDimension.JointGoalDimensions.value + 1:
                goal = JointGoal()
                goal.from_array(array)
                self.goal_list.append(goal)
            else:
                raise Exception("Array of length %d not supported" % array.shape[0])

    def to_list_of_array(self):
        """
        It returns a list of "well shaped" arrays, decoded from the current trajectory.

        :return: a list of array representing the trajectory
        :rtype: list
        """
        loa = []
        for goal in self.goal_list:
            loa.append(goal.to_array())
        return loa

    def from_np_file(self, filename):
        """
        Recover the trajectory from file.

        :param filename: Name of the file
        :type filename: str
        """
        self.from_list_of_array(np.load(filename))

    def to_np_file(self, filename):
        """
        Save the trajectory on file.

        :param filename: Name of the file
        :type filename: str
        """
        np.save(filename, self.to_list_of_array())

    def get_type(self):
        """
        Get whether the trajectory is just described in Joint space or in Cartesian spaced, or it is mixed.

        :return: CartTrajectoryType | JointTrajectoryType | MixedTrajectoryType
        :rtype: TrajectoryType
        """
        flag = True
        for goal in self.goal_list:
            if goal.dimensions != GoalDimension.CartGoalDimensions.value:
                flag = False
                break

        if flag:
            return TrajectoryType.CartTrajectoryType

        flag = True
        for goal in self.goal_list:
            if goal.dimensions != GoalDimension.JointGoalDimensions.value:
                flag = False
                break

        if flag:
            return TrajectoryType.JointTrajectoryType

        return TrajectoryType.MixedTrajectoryType


