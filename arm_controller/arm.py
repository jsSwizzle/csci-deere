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
        world_segment = PySegment('seg0', 'world', 0.0, 0.0, 0.0, [0.0,0.0,0.0], [0.0,0.0,56.0], None)
        waist_segment = PySegment('seg1', 'waist', 90.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,42.93], 'Z', joint_no=0)
        shoulder_segment = PySegment('seg2', 'shoulder', 30.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,120.0], 'Y', joint_no=1)
        elbow_segment = PySegment('seg3', 'elbow', 35.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,118.65], 'Y', joint_no=2)
        wrist_roll_segment = PySegment('seg4', 'wrist_roll', 140.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,60.028], 'Z', joint_no=4)
        wrist_pitch_segment = PySegment('seg5', 'wrist_pitch', 80.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,30.17], 'Y', joint_no=5)

        self._chain = PyChain()
        myChain.append_segment(world_segment)
        myChain.append_segment(waist_segment)
        myChain.append_segment(shoulder_segment)
        myChain.append_segment(elbow_segment)
        myChain.append_segment(wrist_roll_segment)
        myChain.append_segment(wrist_pitch_segment)

        self._servo_speed = 5.0
        self._claw_value = 0.0
        self._default_claw_value = 80.0
        self._solver = Solver(self._chain)
        self._kit = ServoKit(channels=16)

    def get_pos(self):
        """Calculates and returns current position of the arm.

        Calculates based on current positions of arm servo's, the
        current position of the claw, returning as (x, y, z).

        Return:
            current_xyz {list} -- a list containing the (x, y, z) position of the claw.
            current_rpy {list} -- a list containing the (r, p, y) of the claw.
        """
        current_xyz, current_rpy = self._solver.forward_solve(self._current_angles)
        return current_xyz, current_rpy

    def set_speed(self, ss):
        """Set's the speed at which the servo's move.

        Set's the arm rate of speed at which the servo's move into position.

        Args:
            ss {float} -- Rate of speed on a 1-10 scale: 1 being slowest, 10 being fastest.

        Returns:
            ss {float} -- Returns the new servo speed.
        """
        if ss > 1.0 and ss < 10.0:
            self._servo_speed = ss
        return self._servo_speed

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
        current_angles = self._chain.get_current_values()
        angles = self._solver.inverse_solve(current_angles, [x_pos, y_pos, z_pos], [roll, pitch, yaw])
        i = 0
        for segment in self._chain.segments:
            if segment.joint_no != -1:
                self.set_joint(segment, angles[i])
                i += 1

    def set_default_position(self):
        """Loads the default position for the robot arm.

        Sets each servo to its default position found in the servo_info dictionary
        created during class initialization.
        """
        for segment in self._chain.segments:
            if segment.joint_no != -1:
                self.set_joint(segment, segment.default_value)

    def set_joint(self, segment, value):
        """Moves the specified segment to the given value.

        Arguments:
            segment {PySegment} -- segment to move.
            value {float} -- value to apply to joint.
        """
        # TODO: figure out how the speed/rate can be implemented
        self._kit.servo[segment.joint_no].angle = value
        segment.current_val = value
