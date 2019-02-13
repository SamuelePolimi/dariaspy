from dariaspy.observers import MissingRefException
import numpy as np


class InconsistentException(Exception):

    def __init__(self):
        Exception.__init__(self, "The notification is inconsistent.")


class NamedPoint:

    def __init__(self, *refs):
        self.refs = refs
        self.values = np.zeros(len(self.refs))

    def set(self, **values):
        if len(values.values()) != self.refs:
            raise InconsistentException()
        for k in values:
            if not k in self.refs:
                raise MissingRefException(k)
            indx = self.refs.index(k)
            self.values[indx] = values[k]


class NamedTrajectoryBase:

    def __init__(self, refs, durations, values):
        self.refs = refs
        self.duration = durations
        self.values = values

    def __iter__(self):
        for values, d in zip(self.values, self.duration):
            yield {r: v for r, v in zip(self.refs, values.ravel())}, d

    def save(self, filename):
        ret = {
           'refs': self.refs,
            'duration': self.duration,
            'values': self.values
        }
        np.save(filename, ret)

    def get_sub_trajectory(self, *refs):
        ret = self.get_movement(*refs)
        return NamedTrajectoryBase(refs, self.duration, np.array(ret).T)

    def get_movement(self, *refs):

        ret = []
        for ref in refs:
            indx = self.refs.index(ref)
            ret.append(self.values[:, indx])
        return ret


class GoToTrajectory(NamedTrajectoryBase):

    def __init__(self, duration=10., **values):
        NamedTrajectoryBase.__init__(self, values.keys(),
                                     np.array([duration]), np.array([[values[ref] for ref in values.keys()]]))


def LoadTrajectory(filename):
    obj = np.load(filename)
    return NamedTrajectoryBase(obj.item()['refs'], obj.item()['duration'], obj.item()['values'])


class NamedTrajectory(NamedTrajectoryBase):

    def __init__(self, *refs):
        NamedTrajectoryBase.__init__(self, refs, np.zeros(0), np.zeros((0, len(refs))))

    def notify(self, duration=0.1, **values):
        if len(values.values()) != len(self.refs):
            raise InconsistentException()
        app_values = np.zeros(len(self.refs))
        for k in values:
            if k not in self.refs:
                raise MissingRefException(k)
            indx = self.refs.index(k)
            app_values[indx] = values[k]
        concat = app_values.reshape(1, -1)
        self.values = np.concatenate([self.values, concat], axis=0)
        self.duration = np.concatenate([self.duration, np.array([duration])], axis=0)