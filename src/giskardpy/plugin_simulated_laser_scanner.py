import math
from Queue import Empty, Queue
from time import time
from wx import Point

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from hsrb_interface import Vector3
from py_trees import Status
from scipy.spatial.transform.rotation import Rotation as rot
from sensor_msgs.msg import LaserScan

from giskardpy import identifier
from giskardpy.data_types import LaserCollisions, LaserCollision
from giskardpy.logging import loginfo
from giskardpy.plugin import GiskardBehavior
from giskardpy.pybullet_wrapper import raytracing_pybullet


class SimulatedLaserScanner(GiskardBehavior):
    """
    Listens to a laser scanner topic, transforms it into a dict and writes it to the god map.
    """

    def __init__(self, name):
        super(SimulatedLaserScanner, self).__init__(name)
        self.laser_publisher = rospy.Publisher("simulated_laser", PoseArray, queue_size=1)

    def createpose(self, a):
        pose = Pose()
        pose.position.x = a[0]
        pose.position.y = a[1]
        pose.position.z = a[2]
        pose.orientation.w = 0
        return pose

    @profile
    def update(self):
        # self.logger.info("update called")
        # HSRB Laser Example Message
        # angle_min: -2.09999990463
        # angle_max: 2.09999990463
        # angle_increment: 0.00583333335817
        # time_increment: 0.0
        # scan_time: 0.0
        # range_min: 0.0500000007451
        # range_max: 60.0

        #base_range_sensor_link
        relativ = self.get_robot().get_fk_pose(self.get_robot().get_root(), "base_range_sensor_link")
        relative = relativ.pose.position
        start_pos = []
        end_pos = []
        for f in np.arange(-2, 2, 0.0058):
            start_pos += [[relative.x, relative.y, relative.z + 0.05]]
            vec = Vector3(np.cos(f), np.sin(f), 0.0)
            quat = relativ.pose.orientation
            rotation = rot.from_quat([quat.x, quat.y, quat.z, quat.w])
            vec += rotation.as_rotvec()
            vec *= 60
            #vec += relative #TODO
            end_pos += [vec]
        points_hit = raytracing_pybullet(start_pos, end_pos)
        #print(points_hit)

        collisions = LaserCollisions(self.get_robot(), len(points_hit))

        for i in points_hit:
            distance = np.linalg.norm(np.subtract(Vector3(relative.x, relative.y, relative.z), i))
            if distance > 0.1:
                collisions.add(LaserCollision(i, distance))

        self.god_map.set_data(identifier.laser_data, collisions)

        posearra = map(self.createpose, points_hit)
        par = PoseArray()
        par.poses = posearra
        par.header.frame_id = "map"
        par.header.stamp.secs = rospy.get_time()
        self.laser_publisher.publish(par)

        return Status.RUNNING


def laser_msg_to_stored_data(msg):
    """
    TODO
    """
    return msg
