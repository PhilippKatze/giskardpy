from Queue import Empty, Queue
from time import time

import rospy
from py_trees import Status
from sensor_msgs.msg import LaserScan

from giskardpy import identifier
from giskardpy.plugin import GiskardBehavior


class LaserScanner(GiskardBehavior):
    """
    Listens to a laser scanner topic, transforms it into a dict and writes it to the god map.
    """

    def __init__(self, name, laser_scanner_topic=u'laser_scanner'):
        """
        :type js_identifier: str
        """
        super(LaserScanner, self).__init__(name)
        self.mls = None
        #self.map_frame = self.get_god_map().get_data(identifier.map_frame)
        self.laser_scanner_topic = laser_scanner_topic
        self.lock = Queue(maxsize=1)

    def setup(self, timeout=0.0):
        self.laser_scanner_sub = rospy.Subscriber(self.laser_scanner_topic, LaserScan, self.cb, queue_size=1)
        return super(LaserScanner, self).setup(timeout)

    def cb(self, data):
        try:
            self.lock.get_nowait()
        except Empty:
            pass
        self.lock.put(data)

    def update(self):
        try:
            if self.mls is None:
                ls = self.lock.get()
            else:
                ls = self.lock.get_nowait()
            self.mls = laser_msg_to_stored_data(ls)
        except Empty:
            pass

        #robot_frame = self.get_robot().get_root()
        #base_pose = lookup_pose(self.map_frame, robot_frame)
        #self.get_robot().base_pose = base_pose.pose

        self.god_map.set_data(identifier.laser_data, self.mls)
        return Status.SUCCESS

def laser_msg_to_stored_data(msg):
    """
    TODO
    """
    return msg