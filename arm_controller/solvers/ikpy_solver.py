"""Implementation of Solver Class to solve Kinematics of Arm Class using IKPy library.
"""
import ikpy.utils.geometry
import numpy as np
from ikpy import chain as ikpc
from arm_controller.chains.py_chain import PyChain
from arm_controller.chains.py_segment import PySegment
from arm_controller.solvers.abstract_solver import AbstractSolver, matrix4x4_to_xyz_rpy
import matplotlib.pyplot as plt


class IKPySolver(AbstractSolver):

    def __init__(self, chain):
        """Abstract Kinematic Solver class.
        """
        self.chain = chain
        self._chain = ikpc.Chain(ikpc.URDF.get_urdf_parameters(chain.urdf.path,
                                                               [chain.urdf.links[0].name]))

    def inverse_solve(self, target_coords, target_rpy, **kwargs):
        """
        :param target_coords:
        :param target_rpy:
        :param kwargs:
            :keyword orientation_mode:
        :return: angles_list
        """
        ornt_mode = kwargs['orientation_mode']
        if ornt_mode is None:
            ornt_mode = 'X'
        return self._chain.inverse_kinematics(target_position=target_coords,
                                              target_orientation=target_rpy,
                                              orientation_mode=ornt_mode)

    def forward_solve(self, angles, **kwargs):
        """Finds the (x, y, z, roll, pitch, yaw) position of the end effector of the chain.

        Calculates the current (x, y, z, roll, pitch, yaw) position of the end
        effector of the arm using the given angles of each of the joints.

        Args:
            current_angles {list} -- list of current angles of each rotating joint in the chain.

        Returns:
            coords {list} -- list containing XYZ coordinates of the end effector.
            rpy {list} -- list containing Roll, Pitch, and Yaw of the end effector.
        """
        return matrix4x4_to_xyz_rpy(self._chain.forward_kinematics(angles))

    def segmented_forward_solve(self, angles):
        """
        Finds the (x, y, z) position of every joint in the chain (including the end effector).
        :type angles: object
        :return:
            coords {list} -- 2 dimensional list containing sets of (X, Y, Z) coordinates of each joint.
        """
        coords = []
        matrices = self._chain.forward_kinematics(angles, True)
        for mtx in matrices:
            coords.append(matrix4x4_to_xyz_rpy(mtx)[0])
        return coords
        pass
