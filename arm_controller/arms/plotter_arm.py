"""Plotter class that can be used as a visual representation of a chain/arm.
"""
import os
import enum
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from multiprocessing import Process, Manager
from time import sleep

from arm_controller.arms.abstract_arm import AbstractArm
from arm_controller.solvers.ikpy_solver import IKPySolver
from arm_controller.chains.py_chain import PyChain
from arm_controller.chains.py_segment import PySegment

class PlotterArm(AbstractArm):
    def __init__(self):
        """Constructs Plotter class.
        """
        
        dirname = os.path.dirname(__file__)
        filepath = os.path.join(dirname, '../urdf/mechatronics_arm.urdf')
        print(f'{filepath}')
        self._chain = PyChain(urdf_file_path=filepath)

        self._servo_speed = 10.0
        self._solver = IKPySolver(self._chain)

        # variables animation depends on
        self.manager = Manager()
        self.anim_variables = self.manager.dict()
        self.anim_variables['exit'] = False # variable to tell animation to exit

        for segment in self._chain.segments:
            if segment.joint_no != -1:
                self.anim_variables[segment.id] = 0.0

        self.proc = Process(target=run_animation, args=(self.anim_variables, self._solver))
        self.proc.start()

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

    def set_speed(self, ss):
        """Set's the speed at which the servo's move.

        Set's the arm rate of speed at which the servo's move into position.

        Args:
            ss {float} -- Rate of speed on a 1-10 scale: 1 being slowest, 10 being fastest.

        Returns:
            ss {float} -- Returns the new servo speed.
        """
        if ss > 1.0:
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
        for segment in self._chain.segments[::-1]:
            if segment.joint_no != -1:
                self.set_joint(segment, segment.default_value)

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
                self.anim_variables[segment.id] = current_value
                sleep(0.5)
            if((current_value - target_value) != 0.0):
                self.anim_variables[segment.id] = target_value
        elif(target_value > current_value):
            while((target_value - current_value) >= step):
                current_value = current_value + step
                self.anim_variables[segment.id] = current_value
                sleep(0.5)
            if((target_value - current_value) != 0.0):
                self.anim_variables[segment.id] = target_value

        segment.current_val = value

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
        for key in anim_variables.keys():
            if key != 'exit':
                angles.append(anim_variables[key])

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

    anim = FuncAnimation(fig, func=animate, frames=300, interval=17, repeat=True, blit=True)
    plt.draw()
    while not anim_variables['exit']:
        plt.pause(0.01)
    plt.close()
    print('Exiting Animation Process')
