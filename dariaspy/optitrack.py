#!/usr/bin/env python
"""
This module (currently empty) is needed to exposes frames relative to objects detected by the optitrack.

"""
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import rospy
import time

from ros_listener import RosListener, activate_listener
from darias_space import CartGoal


class NotificationMissing(Exception):

    def __init__(self):
        Exception.__init__(self, "The frame has not been notified yet.")


class NotExistingFrame(Exception):

    def __init__(self, s):
        Exception.__init__(self, "The frame %s does not exist or has not been yet recognized." % s)


class Frame:

    def __init__(self, tf_object):
        """
        This object represent a frame. Every time we notify a new tf frames, we can refresh this object.

        :param tf_object:
        :type tf_object: TransformStamped
        """
        self.tf_object = tf_object
        self._last_update = time.time()
        self.duration = None

    def notify(self, tf_object):
        """
        Notify a new transformation object.

        :param tf_object:
        :type TransformStamped:
        """
        now = time.time()
        self.duration = now - self._last_update
        self.tf_object = tf_object
        self._last_update = now

    # TODO: use named point
    @DeprecationWarning
    def get_goal(self):
        """
        Provide a cartesian goal computed from the last frame update.

        :return: A goal representing the last movement of the frame.
        :rtype: CartGoal
        """
        if self.duration is not None:
            return CartGoal(self.tf_object.transform, duration=self.duration)
        else:
            raise NotificationMissing()


class TFFrames(RosListener):
    """
    Access to the objects
    """

    def __init__(self):
        RosListener.__init__(self)
        activate_listener()
        rospy.Subscriber('/tf', TFMessage,
                         self.get_callback())
        self.frames = {}

    def _callback(self, data):
        """
        Callback of TFTree

        :param data:
        :type data: TFMessage
        :return:
        """
        for tf in data.transforms:
            if tf.child_frame_id not in self.frames:
                self.frames[tf.child_frame_id] = Frame(tf)
            else:
                self.frames[tf.child_frame_id].notify(tf)

    def get_frame(self, name, reference=None, max_wait=1.):
        """
        This method provide the access to a frame, as soon as the information are available, and return a goal.

        :param name: Name of the frame.
        :param reference: Frame of reference. Compute the relative transformation between frames. Still to be implemented!!
        :param max_wait: Maximum waiting time expressed in seconds. After this, we assume the frame does not exists and
        and an error is arised.
        :return:
        :rtype: CartGoal
        """
        start_time = time.time()

        while name not in self.frames and time.time() - start_time < max_wait:
            pass

        if name not in self.frames:
            raise NotExistingFrame(name)

        while time.time() - start_time < max_wait:
            try:
                return self.frames[name].get_goal()
            except NotificationMissing:
                pass

        return self.frames[name].get_goal()
