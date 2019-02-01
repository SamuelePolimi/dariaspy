"""
This module takes care of observing a part of interest in a system
"""


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

    def __init__(self, *refs):
        self.refs = {}
        for ref in refs: self.add(ref)

    def add(self, ref_info):
        if ref_info.ref_id in self.refs:
            raise  DuplicateRefException(self.refs[ref_info.ref_id], ref_info)

        self.refs[ref_info.ref_id] = ref_info


observable_refs = ObservableRefs(
    *[RefInfo("%s_%s%s" % (a, r, c), "Coordinate %s of %s of the end-effector of the  %s arm." % (c_s, r_s, a_s), False)
                for a, a_s in zip('RL',['right', 'left'])
                for r, r_s in zip('TR', ["traslation", "rotation"])
                for c, c_s in zip('XYZ', 'xyz')]
)


class Observer:

    def __init__(self):
        pass

    def __call__(self, *ref_list):
        raise NotImplementedError

    def get_possible_refs(self):
        raise NotImplementedError


class JointObserver(Observer):

    def __init__(self, *observers):
        Observer.__init__(self)
        self.observers = observers

    def __call__(self, *ref_list):
        ret = {}
        for ref in ref_list:
            for observer in self.observers:
                if ref in observer.get_possible_refs():
                    ret[ref] = observer(ref)
                    break
            raise MissingRefException(ref)
        return ret

    def get_possible_refs(self):
        ret = set()
        for observer in self.observers:
            ret = ret.union(observer.get_possible_refs())
        return ret


class DariasObserver(Observer):

    def __init__(self, darias):
        Observer.__init__(self)
        self.darias = darias

    def __call__(self, *ref_list):
        return {ref:self.darias.arms.info[ref] for ref in ref_list}

    def get_possible_refs(self):
        return self.darias.arms.order


class EndEffectorObserver(Observer):

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