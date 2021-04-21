"""Test classes to perform unit testing on all class methods
   with return values in robot arm library.
"""
import unittest
from arm_controller.arms import *
from arm_controller.solvers import *
from arm_controller.chains.py_chain import PyChain
from arm_controller.chains.py_segment import PySegment

class Py_Chain(unittest.TestCase):
    """Unit testing class for py_chain class methods
    """
    def test_number_of_segments(self):
        # ToDo determine test cases
        world_segment = PySegment('seg0', 'world', 0.0, 0.0, 0.0, [0.0,0.0,0.0], [0.0,0.0,56.0], None)
        waist_segment = PySegment('seg1', 'waist', 90.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,42.93], 'Z', joint_no=0)
        shoulder_segment = PySegment('seg2', 'shoulder', 150.0, 15.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,120.0], 'Y', joint_no=1)
        elbow_segment = PySegment('seg3', 'elbow', 10.0, 0.0, 60.0, [0.0,0.0,0.0], [0.0,0.0,118.65], 'Y', joint_no=2)
        wrist_roll_segment = PySegment('seg4', 'wrist_roll', 90.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,60.028], 'Z', joint_no=4)
        wrist_pitch_segment = PySegment('seg5', 'wrist_pitch', 80.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,30.17], 'Y', joint_no=5)

        self._chain = PyChain()
        self._chain.append_segment(world_segment)
        self._chain.append_segment(waist_segment)
        self._chain.append_segment(shoulder_segment)
        self._chain.append_segment(elbow_segment)
        self._chain.append_segment(wrist_roll_segment)
        self._chain.append_segment(wrist_pitch_segment)

        self.assertEqual(self._chain.number_of_segments(), 6)

    def test_push_segment(self):
        #ToDo determine test cases
        test_segment = PySegment('seg1', 'test', 0.0, 0.0, 0.0, [0.0,0.0,0.0], [0.0,0.0,56.0], None)
        world_segment = PySegment('seg0', 'world', 0.0, 0.0, 0.0, [0.0,0.0,0.0], [0.0,0.0,56.0], None)
        self._chain = PyChain()
        self._chain.append_segment(world_segment)
        self._chain.push_segment(test_segment)

        self.assertEqual(len(self._chain.segments), 2)
        self.assertEqual(self._chain.segments[0], test_segment)

    def test_append_segment(self):
        #ToDo determine test cases
        self._chain = PyChain()
        test_segment = PySegment('seg1', 'test', 0.0, 0.0, 0.0, [0.0,0.0,0.0], [0.0,0.0,56.0], None)
        self._chain.append_segment(test_segment)

        self.assertEqual(self._chain.number_of_segments(), 1)
        self.assertEqual(self._chain.segments[0], test_segment)

    def test_get_current_values(self):
        #ToDo determine test cases
        self._chain = PyChain()
        test_segment = PySegment('seg1', 'test', 0.0, 0.0, 0.0, [0.0,0.0,0.0], [0.0,0.0,56.0], 'X')
        self._chain.append_segment(test_segment)

        test_val = self._chain.get_current_values()
        self.assertEqual(test_val, [0.0])

class Arm(unittest.TestCase):
    """Unit testing class for arm class methods
    """
    # def test_move_to(self):
    #     #ToDo determine test cases
        
    def test_get_pos(self):
        #ToDo determine test cases
        world_segment = PySegment('seg0', 'world', 0.0, 0.0, 0.0, [0.0,0.0,0.0], [0.0,0.0,56.0], None)
        waist_segment = PySegment('seg1', 'waist', 90.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,42.93], 'Z', joint_no=0)
        shoulder_segment = PySegment('seg2', 'shoulder', 150.0, 15.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,120.0], 'Y', joint_no=1)
        elbow_segment = PySegment('seg3', 'elbow', 10.0, 0.0, 60.0, [0.0,0.0,0.0], [0.0,0.0,118.65], 'Y', joint_no=2)
        wrist_roll_segment = PySegment('seg4', 'wrist_roll', 90.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,60.028], 'Z', joint_no=4)
        wrist_pitch_segment = PySegment('seg5', 'wrist_pitch', 80.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,30.17], 'Y', joint_no=5)

        self._chain = PyChain()
        self._chain.append_segment(world_segment)
        self._chain.append_segment(waist_segment)
        self._chain.append_segment(shoulder_segment)
        self._chain.append_segment(elbow_segment)
        self._chain.append_segment(wrist_roll_segment)
        self._chain.append_segment(wrist_pitch_segment)

        self._servo_speed = 10.0
        self._claw_joint_no = 6
        self._claw_value = 0.0
        self._default_claw_value = 80.0

class Solver(unittest.TestCase):
    """Unit testing class for solver methods
    """
    from arm_controller.solvers.pykdl_solver import PyKDLSolver
    def test_forward_solve(self):
        world_segment = PySegment('seg0', 'world', 0.0, 0.0, 0.0, [0.0,0.0,0.0], [0.0,0.0,56.0], None)
        waist_segment = PySegment('seg1', 'waist', 90.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,42.93], 'Z', joint_no=0)
        shoulder_segment = PySegment('seg2', 'shoulder', 150.0, 15.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,120.0], 'Y', joint_no=1)
        elbow_segment = PySegment('seg3', 'elbow', 10.0, 0.0, 60.0, [0.0,0.0,0.0], [0.0,0.0,118.65], 'Y', joint_no=2)
        wrist_roll_segment = PySegment('seg4', 'wrist_roll', 90.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,60.028], 'Z', joint_no=4)
        wrist_pitch_segment = PySegment('seg5', 'wrist_pitch', 80.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,30.17], 'Y', joint_no=5)

        self._chain = PyChain()
        self._chain.append_segment(world_segment)
        self._chain.append_segment(waist_segment)
        self._chain.append_segment(shoulder_segment)
        self._chain.append_segment(elbow_segment)
        self._chain.append_segment(wrist_roll_segment)
        self._chain.append_segment(wrist_pitch_segment)

        self._servo_speed = 10.0
        self._claw_joint_no = 6
        self._claw_value = 0.0
        self._default_claw_value = 80.0

        self._solver = PyKDLSolver(self._chain)

        test_val = self._solver.forward_solve(self._chain.get_current_values)
        print(test_val)

if __name__ == '__main__':
    """Runs unit tests.
    """
    unittest.main()
