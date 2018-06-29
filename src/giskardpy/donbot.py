from giskardpy.robot_ros import RobotRos


class DonBot(RobotRos):
    def __init__(self, default_joint_velocity=0.5, urdf_path='iai_donbot.urdf', urdf_str=None, tip='gripper_tool_frame'):
        if urdf_str is None:
            with open(urdf_path, 'r') as file:
                urdf_str = file.read()
        super(DonBot, self).__init__(urdf_str=urdf_str, root_link='base_footprint', tip_links=[tip],
                                     default_joint_velocity=default_joint_velocity)