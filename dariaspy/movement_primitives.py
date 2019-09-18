import numpy as np

from dariaspy.groups import Group
from dariaspy.trajectory import GoToTrajectory


def rbf(centers, width):
    b = lambda x: np.array([np.exp(-(x - c_i) ** 2 / (2 * h_i)) for c_i, h_i in zip(centers, width)]).T  # eq 7
    return lambda x: b(x) / np.sum(b(x), axis=1, keepdims=True)  # eq 8


class MovementPrimitive:

    def __init__(self, group, centers, bandwidths, parameters):
        """
        :param group: On which group do you want to set your promp
        :type group: Group
        """
        self.group = group
        self.centers = centers
        self.bandwidths = bandwidths
        self.params = parameters
        self.phi = rbf(self.centers, self.bandwidths)

    def get_init_trajectory(self, duration=10.):
        z = np.array([0.])
        y = {ref: np.asscalar(np.matmul(self.phi(z), self.params[ref])) for ref in self.group.refs}
        return GoToTrajectory(duration, **y)


def LearnTrajectory(group, trajectory, n_features=10, h=0.5, reg=1E-12):
    t = np.cumsum(trajectory.duration)
    t = (t - t[0]) / (t[-1] - t[0])

    bandwidths = np.repeat([h], n_features, axis=0)
    centers = np.linspace(-2 * h, (1 + 2 * h), n_features)
    l = reg

    phi = rbf(centers, bandwidths)

    Phi = phi(t)

    regr = np.matmul(np.linalg.inv(np.matmul(Phi.T, Phi) + l * np.eye(n_features)),
                           Phi.T)
    w = {ref: np.matmul(regr, y) for ref, y in zip(trajectory.refs, trajectory.values.T)}

    return MovementPrimitive(group,centers, bandwidths, w)
