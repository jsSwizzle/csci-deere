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
        # TODO: update dictionaries according to what is needed for solver
        joint_info = {}
        joint_info['jt1'] = {'function':'waist','default_value':90.0, 'min_value':0.0, 'max_value':180.0,}
        joint_info['jt2'] = {'function':'shoulder','default_value':150.0, 'min_value':0.0, 'max_value':180.0}
        joint_info['jt3'] = {'function':'elbow','default_value':35.0, 'min_value':0.0, 'max_value':180.0,}
        joint_info['jt4'] = {'function':'wrist_roll','default_value':140.0, 'min_value':0.0, 'max_value':180.0,}
        joint_info['jt5'] = {'function':'wrist_pitch','default_value':85.0, 'min_value':0.0, 'max_value':180.0}
        joint_info['jt6'] = {'function':'grip','default_value':80.0, 'min_value':0.0, 'max_value':180.0}
        # TODO: determine if claw grip should be in joint info or seperate as its not needed for solver
        self._joint_info = joint_info

        segment_info = {}
        segment_info['seg1'] = {'head_joint':'jt1','segment_length':10.0}
        segment_info['seg2'] = {'head_joint':'jt2','segment_length':120.0}
        segment_info['seg3'] = {'head_joint':'jt3','segment_length':1.0}
        segment_info['seg4'] = {'head_joint':'jt4','segment_length':1.0}
        segment_info['seg5'] = {'head_joint':'jt5','segment_length':1.0}
        self._segment_info = segment_info

        current_angles = {}
        current_angles['jt1'] = 0.0
        current_angles['jt2'] = 0.0
        current_angles['jt3'] = 0.0
        current_angles['jt4'] = 0.0
        current_angles['jt5'] = 0.0
        current_angles['jt6'] = 0.0
        # TODO: determine if claw grip (jt6) should be stored in current angles to make fk/ik solution returns neater
        self._current_angles = current_angles

        self._servo_speed = 1.0
        self._solver = Solver(segment_info, joint_info)
        self._kit = ServoKit(channels=16)
        self.configure_board()

    def move_to(self, x_pos, y_pos, z_pos, roll=0, pitch=0, yaw=0):
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
        angles = self._solver.specific_inverse_solve(self._current_angles, x_pos, y_pos, z_pos, roll, pitch, yaw)
        for joint in angles:
            self.set_part(joint, angles[joint]['final_angle'])

    def get_pos(self):
        """Calculates and returns current position of the arm.

        Calculates based on current positions of arm servo's, the
        current position of the claw, returning as (x, y, z).

        Return:
            A list containing the (x, y, z) position of the claw.
        """
        current_pos = self._solver.forward_solve(self._current_angles)
        return current_pos

    def set_speed(self, ss):
        """Set's the speed at which the servo's move.

        Set's the arm rate of speed at which the servo's move into position.

        Args:
            ss {float} -- Rate of speed on a 1-10 scale: 1 being slowest, 10 being fastest.
        """
        if ss > 1.0 and ss < 10.0:
            self._servo_speed = ss

    def configure_board(self):
        """Sets mapping for Servo ID to Servo Number.
        """
        joint_no = 0
        for joint in self._joint_info:
            self._joint_info[joint]['joint#'] = servo_no
            servo_no += 1

    def set_default_position(self):
        """Loads the default position for the robot arm.

        Sets each servo to its default position found in the servo_info dictionary
        created during class initialization.
        """
        for joint, info in self._joint_info.items():
            self.set_part(joint, info['default_value'])

    def set_part(self, part, value):
        """Moves the specified part.

        moves the specified part to the given value.

        Arguments:
            part {str} -- item part to move
            value {float} -- value to apply to part
        """
        # TODO: figure out how the speed/rate can be implemented in setting a part to its value
        self._kit.servo[self._joint_info[part]['servo#']].angle = value
        self._current_angles[part] = value
