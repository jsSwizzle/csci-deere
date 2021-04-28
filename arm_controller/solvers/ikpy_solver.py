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
        """Finds the angles for each joint of the arm given a target end effector.

        Calculates the angles each joint needs to be at given the target end
        effector (x_pos, y_pos, z_pos, roll, pitch, yaw) in cartesion space.

        Args:
            target_coords (list[float]): target end effector XYZ coordinates.
            target_rpy (list[float]): target end effector Roll, Pitch, and Yaw.
            **kwargs:
                orientation_mode (str): which axis to focus on in solution {'X', 'Y', 'Z'}
                defaults to 'X'
        Returns:
            angles (list[float]): list of angles for each rotating joint in the chain.
        """
        if 'orientation_mode' in kwargs:
            ornt_mode = kwargs['orientation_mode']
        else:
            ornt_mode = 'X'
        return self._chain.inverse_kinematics(target_position=target_coords,
                                              target_orientation=target_rpy,
                                              orientation_mode=ornt_mode)

    def forward_solve(self, angles, **kwargs):
        """Finds the (x, y, z, roll, pitch, yaw) position of the end effector of the chain.

        Calculates the current (x, y, z, roll, pitch, yaw) position of the end
        effector of the arm using the given angles of each of the joints.

        Args:
            angles (list[float]): list of current angles of each rotating joint in the chain.
            **kwargs:
        Returns:
            coords (list[float]): list containing XYZ coordinates of the end effector.
            rpy (list[float]): list containing Roll, Pitch, and Yaw of the end effector.
        """
        return matrix4x4_to_xyz_rpy(self._chain.forward_kinematics(angles))

    def segmented_forward_solve(self, angles):
        """Finds the (x, y, z) position of every joint in the chain (including the end effector).

        Args:
            angles (list[float]): list of current angles of each rotating joint in the chain.
        Returns:
            coords (list): 2 dimensional list containing sets of (X, Y, Z) coordinates of each joint.
        """
        coords = []
        matrices = self._chain.forward_kinematics(angles, True)
        for mtx in matrices:
            coords.append(matrix4x4_to_xyz_rpy(mtx)[0])
        return coords
        pass

if __name__ == '__main__':
    chain = PyChain(urdf_file_path='../urdf/ex.urdf')
    solver = IKPySolver(chain)
    ik = solver.inverse_solve(target_coords=[.06,.06,.09],target_rpy=[0,90,0], orientation_mode='Y')
    fk = solver.forward_solve(angles=ik)
    print(ik)
    print(fk)