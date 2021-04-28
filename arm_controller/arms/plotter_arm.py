"""Plotter class that can be used as a visual representation of a chain/arm.
"""
import os
import math
from time import sleep
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from multiprocessing import Process, Manager

from arm_controller.arms.abstract_arm import AbstractArm
from arm_controller.solvers.ikpy_solver import IKPySolver
from arm_controller.chains.py_chain import PyChain

class PlotterArm(AbstractArm):
    def __init__(self):
        """Constructs Plotter class.
        """

        dirname = os.path.dirname(__file__)
        filepath = os.path.join(dirname, '../urdf/mechatronics_arm.urdf')
        self._chain = PyChain(urdf_file_path=filepath)

        self._servo_speed = math.radians(20)
        self._solver = IKPySolver(self._chain)

        # variables animation depends on
        self.manager = Manager()
        self.anim_variables = self.manager.dict()
        self.anim_variables['exit'] = False # variable to tell animation to exit

        for joint in self._chain.joints:
            self.anim_variables[joint] = self._chain.joints[joint]['current_value']

        self.proc = Process(target=run_animation, args=(self.anim_variables, self._solver))
        self.proc.start()

        self.set_default_position()

    def exit(self):
        self.anim_variables['exit'] = True

    def get_pos(self):
        """Calculates and returns current position of the arm.

        Calculates based on current positions of arm servo's, the
        current position of the claw, returning as (x, y, z).

        Return:
            current_xyz {list} -- a list containing the (x, y, z) position of the claw.
            PlotterArmcurrent_rpy {list} -- a list containing the (r, p, y) of the claw.
        """
        current_angles = self._chain.get_current_values()
        current_xyz, current_rpy = self._solver.forward_solve(current_angles)
        return current_xyz, current_rpy

    def set_speed(self, ss, radians=False):
        """Set's the speed at which the servo's move.

        Set's the arm rate of speed at which the servo's move into position.

        Args:
            ss {float} -- Rate of servo speed in degrees per second.

        Returns:
            ss {float} -- Returns the new servo speed in radians per second.
            radians {bool} -- Whether the servo speed is given in radians or degrees per second.
        """
        if ss > 1.0 and not radians:
            self._servo_speed = math.radians(ss)
        elif ss > 1.0 and radians:
            self._servo_speed = ss
        return self._servo_speed

    def move_to(self, x_pos, y_pos, z_pos, roll=0, pitch=0, yaw=0, radians=False):
        """Moves the arm to the specified position.

        Calculates and moves the arm so the claw is centered at the
        position (x_pos, y_pos, z_pos).

        Args:
            x_pos {float} -- Final X position of the claw.
            y_pos {float} -- Final Y position of the claw.
            z_pos {float} -- Final Z position of the claw.
            roll {float} -- Final roll angle of the wrist (default to 0).
            pitch {float} -- Final pitch angle of the wrist (default to 0).
            yaw {float} -- Final yaw angle of the wrist (default to 0).
            radians {bool} -- whether the value given is in radians or degrees.

        Return:
            angles {list} -- list of the angles the arm is being set to (in radians).
        """
        if not radians:
            roll_rad = math.radians(roll)
            pitch_rad = math.radians(pitch)
            yaw_rad = math.radians(yaw)

        angles = self._solver.inverse_solve([x_pos, y_pos, z_pos], [roll_rad, pitch_rad, yaw_rad])

        i = 0
        for joint in self._chain.joints:
            self.set_joint(joint, angles[i], radians=True)
            i += 1

        return angles

    def set_default_position(self):
        """Loads the default position for the robot arm.

        Sets each servo to its default position found in the servo_info dictionary
        created during class initialization.
        """
        self.set_joint('elbow', 0, radians=False)
        self.set_joint('shoulder', 150, radians=False)
        for joint in self._chain.joints:
            self.set_joint(joint, self._chain.joints[joint]['default_value'], radians=True)

    def set_joint(self, joint, value, radians=False):
        """Moves the specified segment to the given value.

        Arguments:
            joint {str} -- joint to move.
            value {float} -- value to apply to joint.
            radians {bool} -- whether the value given is in radians or degrees.
        """
        if value == None:
            return

        if not radians:
            value = math.radians(value)

        target = value
        current = self._chain.joints[joint]['current_value']
        step = self._servo_speed / 2 # divide by two here to allow for half second sleeps

        if (current > target):
            # current angle is LARGER than the target angle so we decrement it to get closer
            while (current - target) > step:
                current = current - step
                self.anim_variables[joint] = current
                sleep(0.5)
            else:
                self.anim_variables[joint] = target

        elif (target > current):
            pass # current angle is SMALLER than the target angle so we increment it to get closer
            while (target - current) > step:
                current = current + step
                self.anim_variables[joint] = current
                sleep(0.5)
            else:
                self.anim_variables[joint] = target
        else:
            # current angle is EQUAL to the target angle
            self.anim_variables[joint] = target

        # failsafe catches
        self.anim_variables[joint] = value
        self._chain.joints[joint]['current_value'] = value

def run_animation(anim_variables, solver):
    """Runs an animation on the given plotter arm.

    Arguments:
        arm {PlotterArm} -- plotter arm to run animation on

    """
    # matplotlib objects
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X Position')
    ax.set_xlim(left=-0.20, right=0.20)
    ax.set_ylabel('Y Position')
    ax.set_ylim(bottom=-0.20, top=0.20)
    ax.set_zlabel('Z Position')
    ax.set_zlim(bottom=-0.05, top=0.2)
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
