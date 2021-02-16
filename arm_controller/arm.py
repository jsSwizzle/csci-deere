"""Implementation of Robot class for Mechatronics arm.

Class is used to control Mechatronics robot arm. Basic methods
are inherited from base Robot class.

    Typical usage example:
    arm = Arm()
    arm.setspeed(5.0)
    arm.getpos()
    arm.moveto(1.2,3.4,5.6)
"""
from robot import Robot
from solver import Solver
from adafruit_servokit import ServoKit

class Arm(Robot):

    def __init__(self):
        """Constructs Arm class.
        """
        servo_info = {}
        servo_info['ss'] = {'function':'speed','default_value':1.0, 'min_value':1.0, 'max_value':10.0}
        servo_info['s1'] = {'function':'waist','default_value':90.0, 'min_value':0.0, 'max_value':180.0}
        servo_info['s2'] = {'function':'shoulder','default_value':150.0, 'min_value':0.0, 'max_value':180.0}
        servo_info['s3'] = {'function':'elbow','default_value':35.0, 'min_value':0.0, 'max_value':180.0}
        servo_info['s4'] = {'function':'wrist_roll','default_value':140.0, 'min_value':0.0, 'max_value':180.0}
        servo_info['s5'] = {'function':'wrist_pitch','default_value':85.0, 'min_value':0.0, 'max_value':180.0}
        servo_info['s6'] = {'function':'grip','default_value':80.0, 'min_value':0.0, 'max_value':180.0}
        self._servo_info = servo_info

        segment_info = {}
        segment_info['seg1'] = {'base_servo':'s1','segment_length':1.0,'axis_of_rotation':'Z'}
        segment_info['seg2'] = {'base_servo':'s2','segment_length':1.0,'axis_of_rotation':'Y'}
        segment_info['seg3'] = {'base_servo':'s3','segment_length':1.0,'axis_of_rotation':'Y'}
        segment_info['seg4'] = {'base_servo':'s4','segment_length':1.0,'axis_of_rotation':'X'}
        segment_info['seg5'] = {'base_servo':'s5','segment_length':1.0,'axis_of_rotation':'Y'}
        self._segment_info = arm_info

        current_angles = {}
        current_angles['s1'] = {'angle':0.0}
        current_angles['s2'] = {'angle':0.0}
        current_angles['s3'] = {'angle':0.0}
        current_angles['s4'] = {'angle':0.0}
        current_angles['s5'] = {'angle':0.0}
        current_angles['s6'] = {'angle':0.0}
        self._current_angles = current_angles

        self._solver = Solver(servo_info, segment_info)
        self._kit = ServoKit(channels=16)
        self.configure_board()

    def move_to(self, x_pos, y_pos, z_pos, roll=0, pitch=0):
        """Moves the arm to the specified position.

        Calculates and moves the arm so the claw is centered at the
        position (x_pos, y_pos, z_pos).

        Args:
            x_pos {float} -- Final X position of the claw.
            y_pos {float} -- Final Y position of the claw.
            z_pos {float} -- Final Z position of the claw.
            roll {float} -- Final roll angle of the wrist (default to 0).
            pitch {float} -- Final pitch angle of the wrist (default to 0).
        """
        angles = self._solver.specific_inverse_solve(x_pos, y_pos, z_pos, roll, pitch)
        for angle in angles:
            self.set_part(angle, angles[angle]['final_angle'])

    def get_pos(self):
        """Calculates and returns current position of the arm.

        Calculates based on current positions of arm servo's, the
        current position of the claw, returning as (x, y, z).

        Return:
            A list containing the (x, y, z) position of the claw.
        """
        pass

    def set_speed(self, ss):
        """Set's the speed at which the servo's move.

        Set's the arm rate of speed at which the servo's move into position.

        Args:
            ss {float} -- Rate of speed on a 1-10 scale: 1 being slowest, 10 being fastest.
        """
        pass

    def configure_board(self):
        """Sets mapping for Servo ID to Servo Number
        """
        servo_no = 0
        for servo in self._servo_info:
            self._servo_info[servo]['servo#'] = servo_no
            servo_no += 1

    def set_default_position(self):
        """Loads the default position for the robot arm.

        Sets the
        """
        for servo, info in self._servo_info.items():
            self.set_part(servo, info['default_value'])

    def set_part(self, part, value):
        """Moves the specified part.

        moves the specified part to the given value.

        Arguments:
            part {str} -- item part to move
            value {float} -- value to apply to part
        """
        if part != 'ss':
            self._kit.servo[self._servo_info[part]['servo#']].angle = value
            self._current_angles[part]['angle'] = value
