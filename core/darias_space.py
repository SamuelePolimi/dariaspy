import numpy as np

CartGoalDimensions = 3
JointGoalDimensions = 22

CartTrajectoryType = 0
JointTrajectoryType = 1
MixedTrajectoryType = 2


class Goal:

    def __init__(self, position, duration, dimensions=7):
        self.position = position if position is not None else np.zeros(dimensions)
        self.duration = duration
        self.dimensions = dimensions

    def _check(self):
        assert len(self.position.shape) == 1 and self.position.shape[0] == self.dimensions, \
            "Position must be a 1-dimensional array of length %d." % self.dimensions
        assert np.isscalar(self.duration), "Duration must be a scalar"

    def from_array(self, array):
        if array.shape[0] == self.dimensions + 1:
            self.position = array[:self.dimensions]
            self.duration = array[-1]
        else:
            raise Exception("Array dimension should be %d, since the first %d are reserved for the position and the last one for the duration" % (self.dimensions + 1, self.dimensions))

    def to_array(self):
        return np.concatenate([self.position, [self.duration]], axis=0)


class CartGoal(Goal):

    def __init__(self, position=None, duration=1):
        Goal.__init__(self, position, duration, CartGoalDimensions)


class JointGoal(Goal):

    def __init__(self, position=None, duration=1):
        Goal.__init__(self, position, duration, JointGoalDimensions)


class Trajectory:

    def __init__(self, goal_list):
        self.goal_list = goal_list

    def from_list_of_array(self, loa):
        self.goal_list = []
        for array in loa:
            if array.shape[0] == CartGoalDimensions + 1:
                goal = CartGoal()
                goal.from_array(array)
                self.goal_list.append(goal)
            elif array.shape[0] == JointGoalDimensions + 1:
                goal = JointGoal()
                goal.from_array(array)
                self.goal_list.append(goal)
            else:
                raise Exception("Array of length %d not supported" % array.shape[0])

    def to_list_of_array(self):
        loa = []
        for goal in self.goal_list:
            loa.append(goal.to_array())
        return loa

    def from_np_file(self, filename):
        self.from_list_of_array(np.load(filename))

    def to_np_file(self, filename):
        np.save(filename, self.to_list_of_array())

    def get_type(self):
        flag = True
        for goal in self.goal_list:
            if goal.dimensions != CartGoalDimensions:
                flag = False
                break

        if flag:
            return CartTrajectoryType

        flag = True
        for goal in self.goal_list:
            if goal.dimensions != JointGoalDimensions:
                flag = False
                break

        if flag:
            return JointTrajectoryType

        return MixedTrajectoryType


