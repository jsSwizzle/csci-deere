"""Test classes to perform unit testing on all class methods
   with return values in robot arm library.
"""
import unittest
from robot import Robot
from solver import Solver
from py_chain import PyChain
from py_segment import PySegment
from adafruit_servokit import ServoKit
from arm import Arm

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

