"""Plotter class that can be used as a visual representation of a chain/arm.
"""
import enum
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from multiprocessing import Process, Manager

from arm_controller.arms.abstract_arm import AbstractArm
from arm_controller.solvers.pykdl_solver import PyKDLSolver
from arm_controller.chains.py_chain import PyChain
from arm_controller.chains.py_segment import PySegment

class PlotterArm(AbstractArm):
    def __init__(self):
        """Constructs Plotter class.
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

        self._servo_speed = 5.0
        self._solver = PyKDLSolver(self._chain)

        # variables animation depends on
        self.manager = Manager()
        self.anim_variables = self.manager.dict()
        self.anim_variables['exit'] = False # variable to tell animation to exit
        self.anim_variables['start_angles'] = [] # starting angles of the current animation
        self.anim_variables['end_angles'] = [] # ending angles of the current animation
        self.anim_variables['is_running'] = True

        self.set_default_position()

        self.proc = Process(target=run_animation, args=(self.anim_variables, self._solver))
        self.proc.start()

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
        for angle in angles:
            if segment.joint_no != -1:
                segment.current_val = angle

        self.anim_variables['start_angles'] = current_angles
        self.anim_variables['end_angles'] = angles
        self.anim_variables['is_running'] = True

    def set_default_position(self):
        """Loads the default position for the robot arm.

        Sets each servo to its default position found in the servo_info dictionary
        created during class initialization.
        """
        current_angles = self._chain.get_current_values()
        self.anim_variables['start_angles'] = current_angles

        angles = []
        for segment in self._chain.segments:
            if segment.joint_no != -1:
                default = segment.default_value
                segment.current_val = default
                angles.append(default)

        self.anim_variables['end_angles'] = angles
        self.anim_variables['is_running'] = True

    def set_joint(self, segment, value):
        """Moves the specified segment to the given value.

        Arguments:
            segment {PySegment} -- segment to move.
            value {float} -- value to apply to joint.
        """
        current_angles = self._chain.get_current_values()
        segment.current_val = value
        angles = self._chain.get_current_values()

        self.anim_variables['start_angles'] = current_angles
        self.anim_variables['end_angles'] = angles
        self.anim_variables['is_running'] = True

def run_animation(anim_variables, solver):
    """Runs an animation on the given plotter arm.

    Arguments:
        arm {PlotterArm} -- plotter arm to run animation on

    """
    # matplotlib objects
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    ax.set_title('3D Plot of Robot Arm')

    # inner function called to animate
    def animate(i):
        angles = []
        for a in range(len(anim_variables['start_angles'])):
            start_angle, end_angle = anim_variables['start_angles'][a], anim_variables['end_angles'][a]
            angle = start_angle + ((end_angle - start_angle) * (i / 499))
            angles.append(angle)
        coords = solver.segmented_forward_solve(angles)

        xdata = []
        ydata = []
        zdata = []

        for c in coords:
            xdata.append(c[0])
            ydata.append(c[1])
            zdata.append(c[2])

        line = ax.plot(xdata, ydata, zdata, c="black", lw=2)
        return line

    anim = FuncAnimation(fig, func=animate, frames=500, interval=10, repeat=False, blit=True)
    anim_variables['is_running'] = False
    plt.draw()
    while not anim_variables['exit']:
        if anim_variables['is_running']:
            anim = FuncAnimation(fig, func=animate, frames=500, interval=10, repeat=False, blit=True)
            anim_variables['is_running'] = False
        plt.pause(0.5)
    plt.close()
    print('Exiting Animation Process')
