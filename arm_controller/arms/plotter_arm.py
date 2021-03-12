"""Plotter class that can be used as a visual representation of a chain/arm.
"""
import enum
import math
import numpy

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from arm_controller.arms.abstract_arm import AbstractArm
from arm_controller.solvers.pykdl_solver import PyKDLSolver
from arm_controller.chains.py_chain import PyChain
from arm_controller.chains.py_segment import PySegment

def calculateDeltas(arr0: [], arr1: []):
    ret = []
    for i, (a0, a1) in enumerate(zip(arr0, arr1)):
        ret.append(a1 - a0)
    return ret


class PlotterArm(AbstractArm):
    def __init__(self):
        """Constructs Plotter class.
        """
        world_segment = PySegment('seg0', 'world', 0.0, 0.0, 0.0, [0.0,0.0,0.0], [0.0,0.0,56.0], None)
        waist_segment = PySegment('seg1', 'waist', 90.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,42.93], 'Z', joint_no=0)
        shoulder_segment = PySegment('seg2', 'shoulder', 30.0, 15.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,120.0], 'Y', joint_no=1)
        elbow_segment = PySegment('seg3', 'elbow', 35.0, 0.0, 60.0, [0.0,0.0,0.0], [0.0,0.0,118.65], 'Y', joint_no=2)
        wrist_roll_segment = PySegment('seg4', 'wrist_roll', 140.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,60.028], 'Z', joint_no=4)
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
        self._ax = plt.axes(projection='3d')

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
        starting_coords = self._solver.segmented_forward_solve(current_angles)
        i = 0
        for segment in self._chain.segments:
            if segment.joint_no != -1:
                self.set_joint(segment, angles[i], starting_coords)
                i += 1

    def set_default_position(self):
        """Loads the default position for the robot arm.

        Sets each servo to its default position found in the servo_info dictionary
        created during class initialization.
        """
        current_angles = self._chain.get_current_values()
        starting_coords = self._solver.segmented_forward_solve(current_angles)
        for segment in self._chain.segments:
            if segment.joint_no != -1:
                self.set_joint(segment, segment.default_value, starting_coords)

    def set_joint(self, segment, value, starting_coords):
        """Moves the specified segment to the given value.

        Arguments:
            segment {PySegment} -- segment to move.
            value {float} -- value to apply to joint.
        """
        # TODO: figure out how the speed/rate can be implemented
        segment.current_val = value
        current_coords = self._solver.segmented_forward_solve(self._chain.get_current_values())

        self.create_lines([starting_coords, current_coords], self._ax)
        plt.show()

    def plot_3Dpts(self, xs: [], ys: [], zs: []):
        ax = plt.axes(projection='3d')
        lines = ax.plot3D(xs, ys, zs)
        plt.setp(lines, color='r', linewidth=2.0, marker='+', mew=1.0, mec='b')
        plt.grid(True)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

    def create_lines(self, coords: [[[]]], ax):
        lines = []
        for c in coords:
            xs = [item[0] for item in c]
            ys = [item[1] for item in c]
            zs = [item[2] for item in c]
            # print(f'{xs}, {ys}, {zs}')
            ln = ax.plot3D(xs, ys, zs)
            plt.setp(ln, marker='+', mec='k', mew=.8)
            lines.append(ln)
        ax.set_xlim3d(-300, 300)
        ax.set_ylim3d(-300, 300)
        ax.set_zlim3d(0, 300)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        return lines

    def create_timelapse(self, coords1: [[]], coords2: [[]], ax, steps: int):
        lines = []
        x0s, x1s = [item[0] for item in coords1], [item[0] for item in coords2]
        xds = calculateDeltas(x0s, x1s)
        y0s, y1s = [item[1] for item in coords1], [item[1] for item in coords2]
        yds = calculateDeltas(y0s, y1s)
        z0s, z1s = [item[2] for item in coords1], [item[2] for item in coords2]
        zds = calculateDeltas(z0s, z1s)

        currSteps = steps - 1
        while currSteps > 0:
            xs, ys, zs = [], [], []
            for i, (x, d) in enumerate(zip(x1s, xds)):
                xs.append(x - (d / steps * currSteps))
            for i, (y, d) in enumerate(zip(y1s, yds)):
                ys.append(y - (d / steps * currSteps))
            for i, (z, d) in enumerate(zip(z1s, zds)):
                zs.append(z - (d / steps * currSteps))
            ln = ax.plot3D(xs, ys, zs)
            plt.setp(ln, marker='.', mec='b', mew=1, alpha=1 - (currSteps / steps))
            lines.append(ln)
            currSteps -= 1
        ln1 =ax.plot3D(x1s, y1s, z1s)
        plt.setp(ln1, marker='x', mec='r', mew=.8)
        lines.append(ln1)
        ln0 = ax.plot3D(x0s, y0s, z0s)
        plt.setp(ln0, marker='+', mec='k')
        lines.append(ln0)
        ax.set_xlim3d(-150, 150)
        ax.set_ylim3d(-150, 150)
        ax.set_zlim3d(0, 300)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        return lines
