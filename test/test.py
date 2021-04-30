"""Test classes to perform unit testing on all class methods
   with return values in robot arm library.
"""
import unittest
import os
import math
from time import sleep
from arm_controller.solvers.ikpy_solver import IKPySolver
from arm_controller.arms.mechatronics_arm import MechatronicsArm
from arm_controller.chains.py_chain import PyChain
from arm_controller.arms.abstract_arm import AbstractArm
from adafruit_servokit import ServoKit
from arm_controller.chains.py_urdf import PyURDF, URDFObject

class Py_Chain(unittest.TestCase):
    """Unit testing class for py_chain class methods
    """
    def test_number_of_segments(self):
        
        self.urdf = PyURDF.parse("../arm_controller/urdf/ex")
        self.joints = {}
        for joint in self.urdf.joints:
            self.joints[joint.name] = {'current_value': 0.0}
        self.set_default_values()

        self.assertEqual(self._chain.number_of_segments(), 5)

    def test_number_of_joints(self):
        self.urdf = PyURDF.parse("../arm_controller/urdf/ex")
        self.joints = {}
        for joint in self.urdf.joints:
            self.joints[joint.name] = {'current_value': 0.0}
        self.set_default_values()

        self.asserEqual(self._chain.number_of_joints(), 4)

    def test_get_current_values(self):
        self.urdf = PyURDF.parse("../arm_controller/urdf/ex")
        self.joints = {}
        for joint in self.urdf.joints:
            self.joints[joint.name] = {'current_value': 0.0}
        self.set_default_values()
        current_values = []
        for joint in self.joints:
            current_values.append(self.joints[joint]['current_value'])

        self.assertEqual(self.joints[0], {'current_value': 0.0})

    def test_set_default_values(self):
        self.urdf = PyURDF.parse("../arm_controller/urdf/ex")
        self.joints = {}
        for joint in self.urdf.joints:
            self.joints[joint.name] = {'current_value': 0.0}
        self.set_default_values()

        self.assertEqual(self.joints[0], {'current_value': 0.0})


    def test_get_current_values(self):
        self.urdf = PyURDF.parse("../arm_controller/urdf/ex")
        self.joints = {}
        for joint in self.urdf.joints:
            self.joints[joint.name] = {'current_value': 0.0}
        self.set_default_values()

        test_val = self._chain.get_current_values()
        self.assertEqual(test_val[0], {'current_value': 0})

class Arm(unittest.TestCase):
    """Unit testing class for arm class methods
    """

    def test_get_pos(self):
        arm = MechatronicsArm()

        test_val = arm.get_pos()

        self.assertEqual(test_val,  self._solver.forward_solve(self.chain.get_current_values))

    def test_set_speed(self):
        arm = MechatronicsArm()

        self.set_speed(1)
        self.assertEquals(arm._servo_speed, math.radians(1))

    def test_move_to(self):
        arm = MechatronicsArm()
        
        test_val = arm.move_to(100, 100, 100, 0, 0, 0)
        self.assertEquals(test_val, arm.get_pos())

    def test_open_claw(self):
        arm = MechatronicsArm()

        arm.close_claw()
        arm.open_claw()
        test_val = arm.chain.joints['claw']
        self.assertEqual(test_val, {'current_value': 80})

    def test_close_claw(self):
        arm = MechatronicsArm()

        arm.close_claw()
        test_val = arm.chain.joints['claw']
        self.assertEqual(test_val, {'current_value': 30})


if __name__ == '__main__':
    """Runs unit tests.
    """
    unittest.main()
