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
        self.n_params = self.n_features * self.n_dim
        self.group = group
        self.centers = centers
        self.bandwidths = bandwidths
        self.phi = rbf(self.centers, self.bandwidths)
        self.l = regularization
        self.equal_phi = True

    def get_phi(self, z, dim=0):
        return self.phi(z)

    def get_block_phi(self, z):
        ret = block_diag(*([self.get_phi(z, dim=0) for i in range(self.n_dim)]*self.n_dim))
        return ret

    def get_displacement(self, z, dim=0):
        return np.zeros_like(z)

    def get_block_displacement(self, z):
        return np.concatenate([self.get_displacement(z, i) for i in range(self.n_dim)])


class ProjectedMovementSpace(MovementSpace):

    def __init__(self, group, centers, bandwidths, projection_matrix, displacement, regularization=1E-12):
        """
        :param group: On which group do you want to set your promp
        :type group: Group
        :param projection_matrix: n_parameters x n_components
        """
        MovementSpace.__init__(self, group, centers, bandwidths, regularization)
        self.n_params = projection_matrix.shape[1]
        self.projection_matrix = projection_matrix
        self.displacement = displacement
        self.equal_phi = False

    def get_phi(self, z, dim=0):
        omega = self.projection_matrix[self.n_features*dim, self.n_features*(dim+1), :]
        return self.phi(z) @ omega

    def get_displacement(self, z, dim=0):
        return self.displacement[self.n_features*dim: self.n_features*(dim+1)]


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


class PrincipalMovementPrimitive:

    def __init__(self, movement_space, movement_primitives):
        """

        :param movement_space: the space in which the movement is carried
        :param movement_primitives: list of the movement primitives
        :type movement_space: MovementSpace
        :type movement_primitives: list[MovementPrimitive]
        """
        self.movement_space = movement_space
        self.movement_timitives = movement_primitives
        self._params = np.array([primitive.get_block_params() for primitive in movement_primitives])
        self._mean_params = np.mean(self._params, axis=0)
        self._cov_params = np.cov(self._params)


def ClassicSpace(group, n_features=10, regularization=1E-12):
    h = 1/(np.sqrt(2.*n_features)*n_features)                       # TODO: why?
    bandwidths = np.repeat([h], n_features, axis=0)
    centers = np.linspace(-2 * h, (1 + 2 * h), n_features)
    return MovementSpace(group, centers, bandwidths, regularization/n_features)


def LearnTrajectory(movement_space, trajectory):
    """

    :param movement_space:
    :type movement_space: MovementSpace
    :param trajectory:
    :return:
    """
    n = len(trajectory.duration)
    t = np.cumsum(trajectory.duration)
    t = (t - t[0]) / (t[-1] - t[0])

    bandwidths = movement_space.bandwidths
    centers = movement_space.centers
    l = movement_space.l

    if movement_space.equal_phi:
        phi = rbf(centers, bandwidths)

        Phi = phi(t)
        A = np.matmul(Phi.T, Phi) + n * l * np.eye(movement_space.n_features)
        w = {ref: np.linalg.solve(A, np.matmul(Phi.T, y)) for ref, y in zip(trajectory.refs, trajectory.values.T)}
    else:
        Phi = movement_space.get_block_displacement(t)
        A = np.matmul(Phi.T, Phi) + n * l * np.eye(movement_space.n_features)
        y = np.concatenate([t for t in trajectory.values.T], axis=0)
        params = np.linalg.solve(A, np.matmul(Phi.T, (y - movement_space.get_displacement(t))))
        w = {movement_space.group.refs[i]:params[movement_space.n_features*i:movement_space.n_features*(i+1)]
             for i in range(movement_space.n_dim)}

    return MovementPrimitive(movement_space, w)


def eigen_value_decomposition(cov_matrix):
    eigen_vals, eigen_vectors = [np.real(x) for x in np.linalg.eig(cov_matrix)]
    vals = np.abs(eigen_vals.reshape((len(eigen_vals), 1)))
    vectors = np.transpose(eigen_vectors)
    eigen_matrix = np.hstack((vals, vectors))
    eigen_matrix = np.flip(eigen_matrix[eigen_matrix[:, 0].argsort()], axis=0)
    return eigen_matrix


def LearnPrincipalMovements(movement_space, trajectories, n_components=10):
    movements = [LearnTrajectory(movement_space, t) for t in trajectories]
    param_matrix = np.array([mp.get_block_params() for mp in movements])
    mean_param = np.mean(param_matrix, axis=0)
    cov_matrix = np.cov(param_matrix)
    projection = eigen_value_decomposition(cov_matrix)[:n_components, 1:].T
    projection_space = ProjectedMovementSpace(movement_space.group, movement_space.centers, movement_space.bandwidths,
                           projection, mean_param, regularization=movement_space.l)
    return projection_space, [LearnTrajectory(projection_space, t) for t in trajectories]