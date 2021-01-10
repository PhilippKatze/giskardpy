import math
from Queue import Empty, Queue
from time import time

import rospy
from py_trees import Status
from sensor_msgs.msg import LaserScan

from giskardpy import identifier
from giskardpy.logging import loginfo
from giskardpy.plugin import GiskardBehavior


class LaserScanner(GiskardBehavior):
    """
    Listens to a laser scanner topic, transforms it into a dict and writes it to the god map.
    """

    def __init__(self, name, laser_scanner_topic=u'/hsrb/base_scan'):
        super(LaserScanner, self).__init__(name)
        self.mls = None
        self.laser_scanner_topic = laser_scanner_topic
        self.lock = Queue(maxsize=1)
        #self.logger.info("scanner inited")

    def setup(self, timeout=0.0):
        self.laser_scanner_sub = rospy.Subscriber(self.laser_scanner_topic, LaserScan, self.cb, queue_size=1)
        #self.logger.info("scanner sub created topic : "+self.laser_scanner_topic)
        return super(LaserScanner, self).setup(timeout)

    def cb(self, data):
        #self.logger.info("cb called")
        try:
            self.lock.get_nowait()
        except Empty:
            pass
        self.lock.put(data)

    def update(self):
        #self.logger.info("update called")
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

        #laser_scan = self.god_map.get_data(identifier.laser_data)
        #angle = laser_scan.angle_min

        #for i, range in enumerate(laser_scan.ranges):
        #    if laser_scan.range_min <= range <= laser_scan.range_max:
        #        x = range * math.cos(angle)
        #        y = range * math.sin(angle)
        #        self.logger.info(str(x) + " " + str(y))

        #self.logger.info("updated laser data")
        #self.logger.info(str(self.get_god_map().get_data(identifier.laser_data + ["ranges","0"] )))
        #loginfo()

        return Status.SUCCESS

def laser_msg_to_stored_data(msg):
    """
    TODO
    """
    return msg