import numpy as np
from scipy.linalg import block_diag

from dariaspy.groups import Group
from dariaspy.trajectory import GoToTrajectory, NamedTrajectoryBase


def rbf(centers, width):
    b = lambda x: np.array([np.exp(-(x - c_i) ** 2 / (2 * h_i)) for c_i, h_i in zip(centers, width)]).T  # eq 7
    return lambda x: b(x) / np.sum(b(x), axis=1, keepdims=True)  # eq 8


class MovementSpace:

    def __init__(self, group, centers, bandwidths, regularization=1E-12):
        """
        :param group: On which group do you want to set your promp
        :type group: Group
        """
        self.n_features = len(centers)
        self.n_dim = len(group.refs)
        self.group = group
        self.centers = centers
        self.bandwidths = bandwidths
        self.phi = rbf(self.centers, self.bandwidths)
        self.l = regularization

    def get_block_phi(self, z):
        phi = self.phi(z)
        ret = block_diag(*([phi]*self.n_dim))
        return ret


class MovementPrimitive:

    def __init__(self, movement_space,  parameters):
        """

        """
        self.movement_space = movement_space
        self.params = parameters

    def get_init_trajectory(self, duration=10.):
        z = np.array([0.])
        y = {ref: np.asscalar(np.matmul(self.movement_space.phi(z), self.params[ref]))
             for ref in self.movement_space.group.refs}
        return GoToTrajectory(duration, **y)

    def get_full_trajectory(self, frequency=20, duration=10.):
        ctr_time = 1./frequency
        n_points = int(duration*frequency)
        z = np.linspace(0., 1., n_points)
        phi = self.movement_space.phi(z)
        y = np.array([np.matmul(phi, self.params[ref]) for ref in self.movement_space.group.refs]).T
        return NamedTrajectoryBase(self.movement_space.group.refs, np.array([ctr_time]*n_points), y)

    def get_block_params(self):
        return np.concatenate([self.params[ref] for ref in self.movement_space.group.refs], axis=0)


def ClassicSpace(group, n_features=10, regularization=1E-12):
    h = 2./n_features
    bandwidths = np.repeat([h], n_features, axis=0)
    centers = np.linspace(-2 * h, (1 + 2 * h), n_features)
    return MovementSpace(group, centers, bandwidths, regularization*n_features)


def LearnTrajectory(movement_space, trajectory):
    """

    :param movement_space:
    :type movement_space: MovementSpace
    :param trajectory:
    :return:
    """
    t = np.cumsum(trajectory.duration)
    t = (t - t[0]) / (t[-1] - t[0])

    bandwidths = movement_space.bandwidths
    centers = movement_space.centers
    l = movement_space.l

    phi = rbf(centers, bandwidths)

    Phi = phi(t)

    regr = np.matmul(np.linalg.inv(np.matmul(Phi.T, Phi) + l * np.eye(movement_space.n_features)),
                           Phi.T)
    w = {ref: np.matmul(regr, y) for ref, y in zip(trajectory.refs, trajectory.values.T)}

    return MovementPrimitive(movement_space, w)
