"""Test classes to perform unit testing on all class methods
   with return values in robot arm library.
"""
import unittest
from arm_controller.arms import *
from arm_controller.solvers import *
from arm_controller.chains import *
from adafruit_servokit import ServoKit

class Solver(unittest.TestCase):
    """Unit testing class for solver class methods
    """
    def test_inverse_solve(self):
        #ToDo determine test cases

    def test_forward_solve(self):
        #ToDo determine test cases

    def test_segmented_forward_solve(self):
        #ToDo determine test cases

class Py_Chain(unittest.TestCase):
    """Unit testing class for py_chain class methods
    """
    def test_number_of_segments(self):
        #ToDo determine test cases

    def test_push_segment(self):
        #ToDo determine test cases

    def test_append_segment(self):
        #ToDo determine test cases

    def test_get_current_values(self):
        #ToDo determine test cases

class Arm(unittest.TestCase):
    """Unit testing class for arm class methods
    """
    def test_move_to(self):
        #ToDo determine test cases

    def test_get_pos(self):
        #ToDo determine test cases

    def test_set_speed(self):
        #ToDo determine test cases

    def test_set_default_position(self):
        #ToDo determine test cases

    def test_set_part(self):
        #ToDo determine test cases

if __name__ == '__main__':
    """Runs unit tests.
    """
    unittest.main()
