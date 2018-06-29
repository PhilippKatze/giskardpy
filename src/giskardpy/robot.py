from collections import namedtuple, OrderedDict
from time import time
from warnings import warn

from tf.transformations import quaternion_from_matrix, quaternion_from_euler
from urdf_parser_py.urdf import URDF

from giskardpy import USE_SYMENGINE, BACKEND
from giskardpy.input_system import ControllerInputArray, FrameInput
from giskardpy.qp_problem_builder import HardConstraint, JointConstraint
import numpy as np

if USE_SYMENGINE:
    import giskardpy.symengine_wrappers as spw
else:
    import giskardpy.sympy_wrappers as spw

Joint = namedtuple('Joint', ['symbol', 'velocity_limit', 'lower', 'upper', 'limitless'])


def hacky_urdf_parser_fix(urdf_str):
    fixed_urdf = ''
    delete = False
    black_list = ['transmission']
    black_open = ['<{}'.format(x) for x in black_list]
    black_close = ['</{}'.format(x) for x in black_list]
    for line in urdf_str.split('\n'):
        if len([x for x in black_open if x in line]) > 0:
            delete = True
        if len([x for x in black_close if x in line]) > 0:
            delete = False
            continue
        if not delete:
            fixed_urdf += line + '\n'
    return fixed_urdf


class Robot(object):
    # TODO add joint vel to internal state
    def __init__(self, default_joint_value=0.0, default_joint_weight=0.0001, default_joint_velocity=0.5,
                 urdf_str=None, root_link='base_footprint', tip_links=()):
        self.root_link = None
        self.default_joint_value = default_joint_value
        self.default_joint_weight = default_joint_weight
        self.urdf_robot = None
        self._joints = OrderedDict()
        self.default_joint_vel_limit = default_joint_velocity

        self.frames = OrderedDict()
        self.link_joint_symbols = {}
        self.q_rot = {}
        self.aa_rot = {}
        self._state = OrderedDict()
        self.hard_constraints = OrderedDict()
        self.joint_constraints = OrderedDict()
        self.current_frame = {}
        self.j = {}
        if urdf_str is not None:
            urdf = hacky_urdf_parser_fix(urdf_str)
            self.load_from_urdf_string(urdf, root_link, tip_links)
            for joint_name in self.weight_input.get_float_names():
                self.set_joint_weight(joint_name, default_joint_weight)

    def get_state(self):
        return self._state

    def get_joint_state(self):
        return OrderedDict((k, self.get_state()[k]) for k in self.get_joint_names())

    def get_joint_names(self):
        return self._joints.keys()

    def _update_observables(self, updates):
        self._state.update(updates)

    # @profile
    def set_joint_state(self, new_joint_state):
        self._update_observables(self.joint_states_input.get_update_dict(**new_joint_state))
        fks = self.all_fks(self.get_joint_state())
        for link, fk in fks.items():
            position = fk[:3,3:].T[0]
            quaternion = quaternion_from_matrix(fk).T
            pose = np.concatenate((quaternion, position))
            self._update_observables(self.current_frame[link].get_update_dict(*pose))

    def set_joint_weight(self, joint_name, weight):
        self._update_observables(self.weight_input.get_update_dict(**{joint_name: weight}))

    def get_joint_state_input(self):
        return self.joint_states_input

    def get_joint_limits(self, joint_name):
        return self._joints[joint_name].lower, self._joints[joint_name].upper

    # @profile
    def add_chain_joints(self, root_link, tip_link):
        """
        Returns a dict with joint names as keys and sympy symbols
        as values for all 1-dof movable robot joints in URDF between
        ROOT_LINK and TIP_LINK.

        :param root_link: str, denoting the root of the kin. chain
        :param tip_link: str, denoting the tip of the kin. chain
        :return: dict{str, sympy.Symbol}, with symbols for all joints in chain
        """

        jointsAndLinks = self.urdf_robot.get_chain(root_link, tip_link, True, True, True)
        parentFrame = self.frames[root_link]
        parentq = self.q_rot[root_link]
        for i in range(1, len(jointsAndLinks), 2):
            joint_name = jointsAndLinks[i]
            link_name = jointsAndLinks[i + 1]
            self.current_frame[link_name] = FrameInput('current', link_name)
            # we need this line in order to have every symbol in our state
            self._update_observables(self.current_frame[link_name].get_update_dict(0,0,0,1,0,0,0))
            joint = self.urdf_robot.joint_map[joint_name]

            if joint_name not in self._joints:
                if joint.type in ['fixed', 'revolute', 'continuous', 'prismatic']:
                    self.frames[link_name] = parentFrame
                    self.q_rot[link_name] = parentq
                    # self.aa_rot[link_name] = parentaa

                    self.frames[link_name] *= spw.translation3(*joint.origin.xyz)*spw.rotation3_rpy(*joint.origin.rpy)
                    # self.frames[link_name] *= spw.rotation3_rpy(*joint.origin.rpy)

                    q = spw.quaternion_from_rpy(*joint.origin.rpy)
                    self.q_rot[link_name] = spw.quaternion_multiply(self.q_rot[link_name], q)

                    # axis, angle = spw.axis_angle_from_rpy(*joint.origin.rpy)
                    # aa = axis * angle
                    # self.aa_rot[link_name] = spw.quaternion_multiply(self.q_rot[link_name], q)
                else:
                    raise Exception('Joint type "{}" is not supported by urdf parser.'.format(joint.type))

                if joint.type != 'fixed':
                    self._joints[joint_name] = Joint(spw.Symbol(joint_name),
                                                     min(joint.limit.velocity, self.default_joint_vel_limit),
                                                     joint.limit.lower,
                                                     joint.limit.upper,
                                                     joint.type == 'continuous')

                if joint.type == 'revolute' or joint.type == 'continuous':
                    self.frames[link_name] *= spw.rotation3_axis_angle(spw.vec3(*joint.axis), spw.Symbol(joint_name))
                    q = spw.quaterntion_from_axis_angle(spw.vec3(*joint.axis), spw.Symbol(joint_name))
                    self.q_rot[link_name] = spw.quaternion_multiply(self.q_rot[link_name], q)

                    self.aa_rot[link_name] = self.frames[link_name] * (spw.vec3(*joint.axis)*spw.Symbol(joint_name))

                elif joint.type == 'prismatic':
                    self.frames[link_name] *= spw.translation3(*(spw.point3(*joint.axis) * spw.Symbol(joint_name))[:3])

            parentFrame = self.frames[link_name]
            parentq = self.q_rot[link_name]

    # @profile
    def load_from_urdf(self, urdf_robot, root_link, tip_links, root_frame=None):
        """
        Returns a dict with joint names as keys and sympy symbols
        as values for all 1-dof movable robot joints in URDF between
        ROOT_LINK and TIP_LINKS.

        :param urdf_robot: URDF.Robot, obtained from URDF parser.
        :param root_link: str, denoting the root of the kin. tree
        :param tip_links: str, denoting the tips of the kin. tree
        :return: dict{str, sympy.Symbol}, with symbols for all joints in tree
        """
        self.urdf_robot = urdf_robot
        self.root_link = root_link

        self.frames[root_link] = root_frame if root_frame is not None else spw.eye(4)
        self.current_frame[root_link] = FrameInput('current', root_link)
        if root_frame is not None:
            raise NotImplementedError("quaternion from root_frame not implemented")
        else:
            self.q_rot[root_link] = spw.Matrix([0, 0, 0, 1])
        self.end_effectors = tip_links
        t = time()
        for tip_link in tip_links:
            self.add_chain_joints(root_link, tip_link)
        print('add chain joints took {}'.format(time() - t))

        self.joint_states_input = ControllerInputArray(self.get_joint_names())
        self.weight_input = ControllerInputArray(self.get_joint_names(), suffix='cc_weight')

        for i, (joint_name, joint) in enumerate(self._joints.items()):
            joint_symbol = self.joint_states_input.to_symbol(joint_name)
            weight_symbol = self.weight_input.to_symbol(joint_name)
            self._state[joint_name] = self.default_joint_value
            self._state[self.weight_input.to_str_symbol(joint_name)] = self.default_joint_weight

            if not joint.limitless:
                self.hard_constraints[joint_name] = HardConstraint(lower=joint.lower - joint_symbol,
                                                                   upper=joint.upper - joint_symbol,
                                                                   expression=joint_symbol)

            self.joint_constraints[joint_name] = JointConstraint(lower=-joint.velocity_limit,
                                                                 upper=joint.velocity_limit,
                                                                 weight=weight_symbol)
        t = time()
        self.make_np_frames()
        print('make np frame took {}'.format(time() - t))

    # @profile
    def make_np_frames(self):
        fast_frames = None
        idxs = []
        for f, expression in self.frames.items():
            if fast_frames is None:
                fast_frames = expression
            else:
                fast_frames = fast_frames.col_join(expression)
            idxs.append((f,(len(idxs)*4,(len(idxs)+1)*4)))
        cse = spw.sp.cse(fast_frames)
        free_symbols = fast_frames.free_symbols
        fast_frames = spw.speed_up(fast_frames, free_symbols, backend='lambda')
        idxs = OrderedDict(idxs)
        # @profile
        def fk(js, link):
            return fast_frames(**js)[idxs[link][0]:idxs[link][1],:]
        self.fk = fk
        # @profile
        def all_fks(js):
            fks = fast_frames(**js)
            return {link: fks[i1:i2,:] for (link, (i1, i2)) in idxs.items()}
        self.all_fks = all_fks
        # self.make_jacobian()

    # @profile
    def link_fk(self, link):
        evaled_frame = self.fk(self.get_joint_state(), link)
        return evaled_frame

    def link_fk_quaternion(self, link):
        fk = self.link_fk(link)
        eef_pos = fk[:3,3:].T[0]
        eef_rot = quaternion_from_matrix(fk).T
        return np.concatenate((eef_rot, eef_pos))


    # @profile
    def get_jacobian(self, tip):
        if tip not in self.j:
            print('creating jacobian for {}'.format(tip))
            f = self.frames[tip]
            x = f[0, 3]
            y = f[1, 3]
            z = f[2, 3]

            current_rotation = spw.rot_of(f)
            hack = spw.rotation3_axis_angle([0, 0, 1], 0.0001)
            current_pose_evaluated = self.current_frame[tip].get_expression()
            current_rotation_evaluated = spw.rot_of(current_pose_evaluated)

            axis, angle = spw.axis_angle_from_matrix((current_rotation.T * (current_rotation_evaluated * hack)).T)
            c_aa = current_rotation[:3, :3] * (axis * angle)

            rx = c_aa[0]
            ry = c_aa[1]
            rz = c_aa[2]
            m = spw.Matrix([x, y, z, rx, ry, rz])
            j = m.jacobian(spw.Matrix(self.get_joint_names()))
            cse = spw.sp.cse(j)
            free_symbols = [spw.Symbol(x) for x in self.get_state().keys()]
            self.j[tip] = spw.speed_up(j, free_symbols, backend=BACKEND)
            print('done')
        # warn('sum '+str(self.nr_cses), DeprecationWarning)
        return self.j[tip](**self.get_state())

    # @profile
    def make_jacobian(self):
        jacobians = None
        idxs = []
        for tip in self.frames:
            f = self.frames[tip]
            x = f[0, 3]
            y = f[1, 3]
            z = f[2, 3]

            current_rotation = spw.rot_of(f)
            hack = spw.rotation3_axis_angle([0, 0, 1], 0.0001)
            current_pose_evaluated = self.current_frame[tip].get_expression()
            current_rotation_evaluated = spw.rot_of(current_pose_evaluated)

            axis, angle = spw.axis_angle_from_matrix((current_rotation.T * (current_rotation_evaluated * hack)).T)
            c_aa = current_rotation[:3, :3] * (axis * angle)

            rx = c_aa[0]
            ry = c_aa[1]
            rz = c_aa[2]
            m = spw.Matrix([x, y, z, rx, ry, rz])
            if jacobians is None:
                jacobians = m.jacobian(spw.Matrix(self.get_joint_names()))
            else:
                jacobians = jacobians.col_join(m.jacobian(spw.Matrix(self.get_joint_names())))
            idxs.append((tip, (len(idxs) * 6, (len(idxs) + 1) * 6)))
        cse = spw.sp.cse(jacobians)
        # ssss = jacobians.free_symbols
        # warn('free_symbols 1 '+str(ssss), DeprecationWarning)
        ssss2 = set([spw.Symbol(x) for x in self.get_state().keys()])
        # warn('diff '+str(ssss2.difference(jacobians.free_symbols)))
        fast_jacobians = spw.speed_up(jacobians, ssss2, backend=BACKEND)
        idxs = OrderedDict(idxs)

        # @profile
        # def f(link):
        #     return fast_jacobians(**self.get_state())[idxs[link][0]:idxs[link][1], :]

        # self.get_jacobian = f


    def make_fk_and_jacobian(self):
        pass

    def load_from_urdf_path(self, urdf_path, root_link, tip_links):
        return self.load_from_urdf(URDF.from_xml_file(urdf_path), root_link, tip_links)

    def load_from_urdf_string(self, urdf_strg, root_link, tip_links):
        return self.load_from_urdf(URDF.from_xml_string(urdf_strg), root_link, tip_links)

    def get_name(self):
        return self.__class__.__name__

    def __str__(self):
        return "{}'s state:\n{}".format(self.get_name(),
                                        '\n'.join('{}:{:.3f}'.format(joint_name, value) for joint_name, value in
                                                  self.get_state().items()))