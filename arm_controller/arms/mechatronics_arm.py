"""Implementation of Robot class for Mechatronics arm.

Class is used to control Mechatronics robot arm. Basic methods
are inherited from abstract base Arm class.

    Typical usage example:
    arm = Arm()
    arm.setspeed(5.0)
    arm.getpos()
    arm.moveto(1.2,3.4,5.6)
"""
from time import sleep
from arm_controller.arms.abstract_arm import AbstractArm
from arm_controller.solvers.pykdl_solver import PyKDLSolver
from arm_controller.chains.py_chain import PyChain
from arm_controller.chains.py_segment import PySegment
from adafruit_servokit import ServoKit

class MechatronicsArm(AbstractArm):

    def __init__(self):
        """Constructs Arm class.
        """
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
        self._kit = ServoKit(channels=16)

    def get_pos(self):
        """Calculates and returns current position of the arm.

        Calculates based on current positions of arm servo's, the
        current position of the claw, returning as (x, y, z).

        Returns:
            current_xyz (list[float]): a list containing the (x, y, z) position of the claw.
            current_rpy (list[float]): a list containing the (r, p, y) of the claw.
        """
        current_angles = self._chain.get_current_values()
        current_xyz, current_rpy = self._solver.forward_solve(current_angles)
        return current_xyz, current_rpy

    def set_speed(self, ss):
        """Set's the speed at which the servo's move.

        Set's the arm rate of speed at which the servo's move into position.

        Args:
            ss (float): Rate of speed on a 1-10 scale: 1 being slowest, 10 being fastest.

        Returns:
            ss (float): Returns the new servo speed.
        """
        if ss > 1.0:
            self._servo_speed = ss
        return self._servo_speed

    def move_to(self, x_pos, y_pos, z_pos, roll=0, pitch=0, yaw=0):
        """Moves the arm to the specified position.

        Calculates and moves the arm so the claw is centered at the
        position (x_pos, y_pos, z_pos).

        Args:
            x_pos (float): Final X position of the claw.
            y_pos (float): Final Y position of the claw.
            z_pos (float): Final Z position of the claw.
            roll (float): Final roll angle of the wrist (default to 0).
            pitch (float): Final pitch angle of the wrist (default to 0).
            yaw (float): Final yaw angle of the wrist (default to 0).
        """
        current_angles = self._chain.get_current_values()
        angles = self._solver.inverse_solve(current_angles, [x_pos, y_pos, z_pos], [roll, pitch, yaw])
        i = 0
        for segment in self._chain.segments:
            if segment.joint_no != -1:
                self.set_joint(segment, angles[i])
                i += 1

    def open_claw(self, value=80.0):
        """Opens the claw of the robot arm.

        Opens the claw of the robot arm, the default value is 80 degrees but any
        value between 0.0 and 180.0 can be entered (for best results use 90.0 as maximum for opening).

        Args:
            value (float): degree to set claw servo to (default is 80.0).
        """
        if value > 0.0 or value < 180.0:
            self._kit.servo[self._claw_joint_no].angle = value
            self._claw_value = value

    def close_claw(self, value=30.0):
        """Closes the claw of the robot arm.

        Closes the claw of the robot arm, the default value is 30 degrees but any
        value between 0.0 and 180.0 can be entered (for best results use 30.0 as the minimum for closing).

        Args:
            value (float): degree to set claw servo to (default is 30.0).
        """
        if value > 0.0 or value < 180.0:
            self._kit.servo[self._claw_joint_no].angle = value
            self._claw_value = value

    def set_default_position(self):
        """Loads the default position for the robot arm.

        Sets each servo to its default position found in the servo_info dictionary
        created during class initialization.
        """
        for segment in self._chain.segments[::-1]:
            if segment.joint_no != -1:
                self.set_joint(segment, segment.default_value)
        self.open_claw()

    def set_joint(self, segment, value):
        """Moves the specified segment to the given value.

        Arguments:
            segment {PySegment} -- segment to move.
            value {float} -- value to apply to joint.
        """
        target_value = value
        current_value = segment.current_val
        step = self._servo_speed / 2

        if(current_value > target_value):
            while((current_value - target_value) >= step):
                current_value = current_value - step
                self._kit.servo[segment.joint_no].angle = current_value
                sleep(0.5)
            if((current_value - target_value) != 0.0):
                self._kit.servo[segment.joint_no].angle = target_value
        elif(target_value > current_value):
            while((target_value - current_value) >= step):
                current_value = current_value + step
                self._kit.servo[segment.joint_no].angle = current_value
                sleep(0.5)
            if((target_value - current_value) != 0.0):
                self._kit.servo[segment.joint_no].angle = target_value

        segment.current_val = value
