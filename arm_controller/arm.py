"""Implementation of Robot class for Mechatronics arm.

Class is used to control Mechatronics robot arm. Basic methods
are inherited from base Robot class.

    Typical usage example:
    arm = Arm()
    arm.setspeed(5.0)
    arm.getpos()
    arm.moveto(1.2,3.4,5.6)
"""
from .robot import Robot
from .solver import Solver
from .py_chain import PyChain
from .py_segment import PySegment
from adafruit_servokit import ServoKit

class Arm(Robot):

    def __init__(self):
        """Constructs Arm class.
        """
        world_segment = PySegment('seg0', 'world', 0.0, 0.0, 0.0, [0.0,0.0,0.0], [0.0,0.0,56], None)
        waist_segment = PySegment('seg1', 'waist', 90.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,42.93], 'Z')
        shoulder_segment = PySegment('seg2', 'shoulder', 150.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,120], 'Y')
        elbow_segment = PySegment('seg3', 'elbow', 35.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,118.65], 'Y')
        wrist_roll_segment = PySegment('seg4', 'wrist_roll', 140.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,60.028], 'Z')
        wrist_pitch_segment = PySegment('seg5', 'wrist_pitch', 80.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,30.17], 'Y')
        myChain = PyChain()
        myChain.append_segment(world_segment)
        myChain.append_segment(waist_segment)
        myChain.append_segment(shoulder_segment)
        myChain.append_segment(elbow_segment)
        myChain.append_segment(wrist_roll_segment)
        myChain.append_segment(wrist_pitch_segment)

        self._servo_speed = 1.0
        self._claw_angle = 0.0
        self._default_claw_angle = 80.0
        self._solver = Solver(self._chain)
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
