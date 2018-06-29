#!/usr/bin/env python
import unittest
from collections import OrderedDict
from simplejson import OrderedDict

import PyKDL
import numpy as np

from urdf_parser_py.urdf import URDF

from giskardpy import USE_SYMENGINE
from giskardpy.donbot import DonBot
from giskardpy.pointy_bot import PointyBot
from giskardpy.pr2 import PR2
from giskardpy.robot import hacky_urdf_parser_fix
from kdl_parser import kdl_tree_from_urdf_model

if USE_SYMENGINE:
    import giskardpy.symengine_wrappers as spw
else:
    import giskardpy.sympy_wrappers as spw

PKG = 'giskardpy'

np.random.seed(23)

def get_rnd_joint_state(robot):
    def f(joint):
        ll, ul = robot.get_joint_limits(joint)
        if ll is None:
            return np.random.random() * np.pi * 2
        return (np.random.random() * (abs(ll) + abs(ul))) + ll

    js = {joint: f(joint) for joint in robot.get_joint_state().keys()}
    return js

def make_kdls(urdf, start, end):
    if urdf[-5:] == '.urdf':
        with open(urdf, 'r') as file:
            urdf = file.read()
    r = URDF.from_xml_string(hacky_urdf_parser_fix(urdf))
    tree = kdl_tree_from_urdf_model(r)
    chain = tree.getChain(start, end)
    kdls = {}
    for i in range(chain.getNrOfSegments()):
        link = str(chain.getSegment(i).getName())
        kdls[link] = KDL(tree, start, link)
    return kdls

class KDL(object):
    def __init__(self, tree, start, end):
        self.end = end
        self.chain = tree.getChain(start, self.end)
        self.fksolver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = PyKDL.ChainJntToJacSolver(self.chain)
        self.jacobian = PyKDL.Jacobian(self.chain.getNrOfJoints())
        self.joints = self.get_joints()

    def get_joints(self):
        joints = []
        for i in range(self.chain.getNrOfSegments()):
            joint = self.chain.getSegment(i).getJoint()
            if joint.getType() != 8:
                joints.append(str(joint.getName()))
        return joints

    def get_tip_nr(self, tip=None):
        if tip is None:
            tip = self.end
        for i in range(self.chain.getNrOfSegments()):
            link_name = str(self.chain.getSegment(i).getName())
            if link_name == tip:
                return i

    def fk(self, js_dict, tip=None):
        js = [js_dict[j] for j in self.joints]
        seg_nr = self.get_tip_nr(tip)
        f = PyKDL.Frame()
        joint_array = PyKDL.JntArray(len(js))
        for i in range(len(js)):
            joint_array[i] = js[i]
        self.fksolver.JntToCart(joint_array, f, seg_nr+1)
        return f

    def fk_np(self, js_dict, tip=None):
        f = self.fk(js_dict, tip)
        r = [[f.M[0, 0], f.M[0, 1], f.M[0, 2], f.p[0]],
             [f.M[1, 0], f.M[1, 1], f.M[1, 2], f.p[1]],
             [f.M[2, 0], f.M[2, 1], f.M[2, 2], f.p[2]],
             [0, 0, 0, 1], ]
        return np.array(r)

    def l_to_jnt_array(self, q):
        j = PyKDL.JntArray(len(q))
        for i in range(len(q)):
            j[i] = q[i]
        return j

    def get_jacobian(self, js_dict):
        js = [js_dict[j] for j in self.joints]
        self.jac_solver.JntToJac(self.l_to_jnt_array(js), self.jacobian)
        np_jacobian = []
        for i in range(6):
            row = []
            for j in range(len(js)):
                row.append(self.jacobian[i,j])
            np_jacobian.append(row)
        return np.array(np_jacobian)


class TestFK(unittest.TestCase):
    def set_js(self, robot, joint_names, js):
        r = []
        for i, k in enumerate(joint_names):
            r.append((k, js[i]))
        r = OrderedDict(r)
        robot.set_joint_state(r)


    def compare_fk_jacobian_chain(self, robot, robot_kdls, js):
        for tip, robot_kdl in robot_kdls.items():
            robot.set_joint_state(js)
            eef_pose = robot.link_fk(tip)
            kdl_pose = robot_kdl.fk_np(js)
            np.testing.assert_array_almost_equal(eef_pose, kdl_pose)
            kdl_pose[0,3]=0
            kdl_pose[1,3]=0
            kdl_pose[2,3]=0
            tip_q = np.array(spw.quaternion_make_unique(robot.q_rot[tip]).subs(js).T).astype(float)[0]
            self.assertTrue(tip_q[-1]>=0)
            kdl_q = robot_kdl.fk(js).M.GetQuaternion()
            try:
                np.testing.assert_array_almost_equal(tip_q, kdl_q)
            except AssertionError as e:
                np.testing.assert_array_almost_equal(-tip_q, kdl_q)


            kdl_aa = robot_kdl.fk(js).M.GetRot()
            spw_axis, spw_angle = spw.axis_angle_from_matrix(eef_pose)
            spw_aa = spw_axis*spw_angle
            if spw.nan not in spw_aa:
                np.testing.assert_array_almost_equal([x for x in kdl_aa], np.array(spw_aa).T[0].astype(float))

            kdl_jacobian = robot_kdl.get_jacobian(js)
            sp_jacobian = robot.get_jacobian(tip)
            sp_jacobian = sp_jacobian[:,np.abs(sp_jacobian).sum(axis=0)!=0]
            np.testing.assert_array_almost_equal(kdl_jacobian, sp_jacobian, decimal=3)


    def test_pr2(self):
        kdls = {}
        kdls.update(make_kdls('pr2.urdf', 'base_link', 'l_gripper_tool_frame'))
        kdls.update(make_kdls('pr2.urdf', 'base_link', 'r_gripper_tool_frame'))
        r = PR2()
        for i in range(20):
            js = get_rnd_joint_state(r)
            self.compare_fk_jacobian_chain(r, kdls, js)


    def test_pointy(self):
        kdls = make_kdls('pointy.urdf', 'base_link', 'eef')
        r = PointyBot()

        for i in range(20):
            js = get_rnd_joint_state(r)
            self.compare_fk_jacobian_chain(r, kdls, js)

    def test_base(self):
        kdls = make_kdls('2d_base_bot.urdf', 'base_link', 'eef')
        r = PointyBot(urdf='2d_base_bot.urdf')

        for i in range(20):
            js = get_rnd_joint_state(r)
            self.compare_fk_jacobian_chain(r, kdls, js)

    def test_donbot(self):
        kdls = make_kdls('iai_donbot.urdf', 'base_footprint', 'gripper_tool_frame')
        r = DonBot(urdf_path='iai_donbot.urdf')

        for i in range(3):
            print(i)
            js = get_rnd_joint_state(r)
            self.compare_fk_jacobian_chain(r, kdls, js)


    def test_fixed(self):
        kdls = make_kdls('fixed.urdf', 'base_link', 'eef')
        r = PointyBot(urdf='fixed.urdf', tip='eef')

        for i in range(20):
            js = get_rnd_joint_state(r)
            self.compare_fk_jacobian_chain(r, kdls, js)

if __name__ == '__main__':
    import rosunit

    rosunit.unitrun(package=PKG,
                    test_name='TestFK',
                    test=TestFK)