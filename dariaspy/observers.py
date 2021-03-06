"""
This module takes care of observing a part of interest in a system
"""

try:
    from optitrack import Frame
except:
    print("The system runs without an optitrack system.")


class MissingRefException(Exception):

    def __init__(self, ref_name):
        Exception.__init__(self, "Ref %r is missing." % ref_name)


class DuplicateRefException(Exception):

    def __init__(self, ref_1, ref_2):
        Exception.__init__(self, """Attempt to insert ref 
        
        %s 
        
        but a reference with same ref_id is already present 
        
        %s.""" % (ref_2, ref_1))


class RefInfo:

    def __init__(self, ref_id, help, ros_inherited=True):
        self.ref_id = ref_id
        self.help = help
        self.ros_inferited = ros_inherited

    def __str__(self):
        return "Ref: %r.\nHelp: %s\nRos inherited: %s." % (self.ref_id, self.help, str(self.ros_inferited))

    def __repr__(self):
        return self.__str__()


class ObservableRefs:
    """
    All the possible references with a description.
    """
    def __init__(self, *refs):
        self.refs = {}
        for ref in refs: self.add(ref)

    def add(self, ref_info):
        """

        :param ref_info: add a reference to the observation.
        :type ref_info: RefInfo
        :rtype: None
        """
        if ref_info.ref_id in self.refs:
            raise DuplicateRefException(self.refs[ref_info.ref_id], ref_info)

        self.refs[ref_info.ref_id] = ref_info

# TODO: insert also the joint references
observable_refs = ObservableRefs(
    *[RefInfo("%s_%s%s" % (a, r, c), "Coordinate %s of %s of the end-effector of the  %s arm." % (c_s, r_s, a_s), False)
                for a, a_s in zip('RL', ['right', 'left'])
                for r, r_s in zip('TR', ["traslation", "rotation"])
                for c, c_s in zip('XYZ', 'xyz')]
)


class Observer:

    def __init__(self):
        pass

    def __call__(self, *ref_list):
        """
        Observe a list of references.
        :param ref_list: list of the name of the reference to observe.
        :type ref_list: str
        :return: a dictionary of reference-values
        :rtype: dict
        """
        raise NotImplementedError

    def get_possible_refs(self):
        """
        Retreive all the reference observed by the observer.
        :return:
        """
        raise NotImplementedError


class JointObserver(Observer):
    """
    Observe the union of two observer (i.e., you can observe at the same time Joint and Cartesian space.)
    """

    def __init__(self, *observers):
        Observer.__init__(self)
        self.observers = observers

    def __call__(self, *ref_list):
        ret = {}
        for ref in ref_list:
            found = False
            for observer in self.observers:
                if ref in observer.get_possible_refs():
                    ret[ref] = observer(ref)[ref]
                    found = True
                    break
            if not found:
                raise MissingRefException(ref)
        return ret

    def get_possible_refs(self):
        ret = set()
        for observer in self.observers:
            ret = ret.union(observer.get_possible_refs())
        return ret


class DariasObserver(Observer):
    """
    Observe the entire joint space of Darias, comprehensive of hands.
    """

    def __init__(self, darias):
        Observer.__init__(self)
        self.darias = darias

    def __call__(self, *ref_list):
        return {ref: self.darias.arms.info[ref] for ref in ref_list}

    def get_possible_refs(self):
        return self.darias.arms.order


class RobotObserver(Observer):
    """
    Observe the entire joint space of the robot, comprehensive of hands.
    """

    def __init__(self, robot):
        Observer.__init__(self)
        self.robot = robot

    def __call__(self, *ref_list):
        return {ref: self.robot.arms.info[ref] for ref in ref_list}

    def get_possible_refs(self):
        return self.robot.arms.order


class EndEffectorObserver(Observer):
    """
    Observe only the left and the right end-effectors in cartesian space.
    """
    def __init__(self, darias):
        Observer.__init__(self)
        self.darias = darias

    def __call__(self, *ref_list):
        try:
            return {ref:self._get_ref(ref) for ref in ref_list}
        except MissingRefException as e:
            raise e

    def _get_ref(self, ref):

        if len(ref) != 4: raise MissingRefException(ref)

        if ref[0] == 'L':
            end_effector = self.darias.left_end_effector
        elif ref[0] == 'R':
            end_effector = self.darias.right_end_effector
        else:
            raise MissingRefException(ref)

        if ref[1] != '_': raise MissingRefException(ref)

        if ref[2] == 'T':
            ref_obj = end_effector.translation
        elif ref[2] == 'R':
            ref_obj = end_effector.rotation
        else:
            raise MissingRefException(ref)

        if ref[3] == 'X':
            return ref_obj[0]
        elif ref[3] == 'Y':
            return ref_obj[1]
        elif ref[3] == 'Z':
            return ref_obj[2]
        elif ref[3] == 'W':
            if ref_obj.shape[0] == 4:
                return ref_obj[3]

        raise MissingRefException(ref)

    def get_possible_refs(self):
        return self.darias.groups["ENDEFF_RIGHT_ARM"].refs + self.darias.groups["ENDEFF_LEFT_ARM"].refs


_v2_reader = {
    'X': 0,
    'Y': 1,
    'Z': 2,
    'W': 3
}

_v1_reader = {
    'T': 0,
    'R': 1
}

_refs = (
    "%s_TX",
    "%s_TY",
    "%s_TZ",
    "%s_RX",
    "%s_RY",
    "%s_RZ",
    "%s_RW"
)


class OptitrackObserver(Observer):
    """
        Observe only the left and the right end-effectors in cartesian space.
        """

    def __init__(self, frame_name):
        Observer.__init__(self)
        self.frame = Frame(frame_name)

    def __call__(self, *ref_list):
        ret = {}
        observation = self.frame.get_cartesian_coordinates()
        for ref in ref_list:
            i_1, i_2 = self._getIndexObs(ref)
            ret[ref] = observation[i_1][i_2]
        return ret

    def _getIndexObs(self, ref_name):

        vals = ref_name.split("_")
        v1, v2 = vals[-1][0], vals[-1][1]
        if "_".join(vals[:-1]) != self.frame.frame_name:
            raise MissingRefException(ref_name)

        try:
            return _v1_reader[v1], _v2_reader[v2]
        except:
            raise MissingRefException(ref_name)

    def get_possible_refs(self):
        return [ref % self.frame.frame_name for ref in _refs]

    
class ScaleObserver(Observer):
    """
    Observe the weight measured from DymoScale
    """

    def __init__(self, scale):
        """
        :param scale: The interface of the scale
        """
        Observer.__init__(self)
        self.scale = scale

    def __call__(self, *ref_list):
        ret = {}
        for ref in ref_list:
            if ref == "%s_WEIGHT" % self.scale.get_name():
                ret[ref] = self.scale.get_weight()
            elif ref == "%s_TARE" % self.scale.get_name():
                ret[ref] = self.scale.get_tare()
            else:
                raise MissingRefException(ref)
        return ret

    def get_possible_refs(self):
        return ["%s_WEIGHT" % self.scale.get_name(),
                "%s_TARE" % self.scale.get_name()]
