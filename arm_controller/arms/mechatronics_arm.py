"""Implementation of Robot class for Mechatronics arm.

Class is used to control Mechatronics robot arm. Basic methods
are inherited from abstract base Arm class.

    Typical usage example:
    arm = Arm()
    arm.setspeed(5.0)
    arm.getpos()
    arm.moveto(1.2,3.4,5.6)
"""
import os
import math
from time import sleep
from arm_controller.solvers.ikpy_solver import IKPySolver
from arm_controller.chains.py_chain import PyChain
from arm_controller.arms.abstract_arm import AbstractArm
from adafruit_servokit import ServoKit

class MechatronicsArm(AbstractArm):
    def __init__(self):
        """Constructs Arm class.
        """

        dirname = os.path.dirname(__file__)
        filepath = os.path.join(dirname, '../urdf/mechatronics_arm.urdf')
        self._chain = PyChain(urdf_file_path=filepath)

        self._servo_speed = math.radians(10.0)
        self._solver = IKPySolver(self._chain)

        self._current_claw_value = 0.0

        self._kit = ServoKit(channels=16)
        self.configure_board()

        self.set_default_position()

    def get_pos(self):
        """Calculates and returns current position of the arm.

        Calculates based on current positions of arm servo's, the
        current position of the claw, returning as (x, y, z).

        Return:
            current_xyz {list} -- a list containing the (x, y, z) position of the claw.
            current_rpy {list} -- a list containing the (r, p, y) of the claw.
        """
        current_angles = self._chain.get_current_values()
        current_xyz, current_rpy = self._solver.forward_solve(current_angles)
        return current_xyz, current_rpy

    def set_speed(self, ss, radians=False):
        """Set's the speed at which the servo's move.

        Set's the arm rate of speed at which the servo's move into position.

        Args:
            ss {float} -- Rate of speed on a 1-10 scale: 1 being slowest, 10 being fastest.

        Returns:
            ss {float} -- Returns the new servo speed.
            radians {bool} -- Whether the servo speed is given in radians or degrees per second.
        """
        if ss > 1.0 and not radians:
            self._servo_speed = math.radians(ss)
        elif ss > 1.0 and radians:
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
        angles = self._solver.inverse_solve([x_pos, y_pos, z_pos], [roll, pitch, yaw])
        i = 0
        for joint in self._chain.joints:
            self.set_joint(segment, angles[i])
            i += 1

    def open_claw(self, value=80.0, radians=False):
        """Opens the claw of the robot arm.

        Opens the claw of the robot arm, the default value is 80 degrees but any
        value between 0.0 and 180.0 can be entered (for best results use 90.0 as maximum for opening).

        Args:
            value {float} -- degree to set claw servo to (default is 80.0).
            radians {bool} -- whether the value to close claw to is in radians or degrees.
        """
        if radians:
            value = math.degrees(value)
        if value > 0.0 or value < 180.0:
            self._kit.servo[self._chain.joints['claw']['servo#']].angle = value
            self._chain.joints['claw']['current_value'] = math.radians(value)

    def close_claw(self, value=30.0, radians=False):
        """Closes the claw of the robot arm.

        Closes the claw of the robot arm, the default value is 30 degrees but any
        value between 0.0 and 180.0 can be entered (for best results use 30.0 as the minimum for closing).

        Args:
            value {float} -- degree to set claw servo to (default is 30.0).
            radians {bool} -- whether the value to close claw to is in radians or degrees.
        """
        if radians:
            value = math.degrees(value)
        if value > 0.0 or value < 180.0:
            self._kit.servo[self._chain.joints['claw']['servo#']].angle = value
            self._chain.joints['claw']['current_value'] = math.radians(value)

    def set_default_position(self):
        """Loads the default position for the robot arm.

        Sets each servo to its default position found in the servo_info dictionary
        created during class initialization.
        """
        self.set_joint('elbow', 0, radians=False)
        self.set_joint('shoulder', 150, radians=False)
        for joint in dict(reversed(list(self._chain.joints.items()))):
            self.set_joint(joint, self._chain.joints[joint]['default_value'], radians=True)
        self.open_claw()

    def set_joint(self, joint, value, radians=False):
        """Moves the specified segment to the given value.

        Arguments:
            joint {str} -- joint to move.
            value {float} -- value to apply to joint (in degrees).
            radians {bool} -- whether the value given is in radians or degrees.
        """
        if value == None:
            return

        if radians:
            value = math.degrees(value)

        target_value = value
        current_value = math.degrees(self._chain.joints[joint]['current_value'])
        step = math.degrees(self._servo_speed) / 2

        if(current_value > target_value):
            while((current_value - target_value) >= step):
                current_value = current_value - step
                self._kit.servo[self._chain.joints[joint]['servo#']].angle = current_value
                sleep(0.5)
            if((current_value - target_value) != 0.0):
                self._kit.servo[self._chain.joints[joint]['servo#']].angle = target_value
        elif(target_value > current_value):
            while((target_value - current_value) >= step):
                current_value = current_value + step
                self._kit.servo[self._chain.joints[joint]['servo#']].angle = current_value
                sleep(0.5)
            if((target_value - current_value) != 0.0):
                self._kit.servo[self._chain.joints[joint]['servo#']].angle = target_value

        self._chain.joints[joint]['current_value'] = math.radians(value)

    def configure_board(self, mapping={'base':None,'waist':0,'shoulder':1,'elbow':2,'wrist_roll':3,'wrist_pitch':4,'claw':5}):
        """Configures the joints with information with use with the Adafruit Servokit.

        Arguments:
            mapping {dict} -- mapping from the joint names (same as the URDF model) to their servo number (use None for joints without servos).
        """
        for joint in mapping:
            self._chain.joints[joint]['servo#'] = mapping[joint]
