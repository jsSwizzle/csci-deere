"""Abstract Solver class and requisite methods
"""
from abc import ABC
import numpy as np
from scipy.spatial.transform import Rotation as R
from arm_controller.chains.py_chain import PyChain


def matrix4x4_to_xyz_rpy(matrix):
    """Takes a 4x4 transformation matrix and returns a tuple of the xyz coordinates and rpy values extracted from the matrix

    Args:
        matrix: Transformation matrix to convert into xyz rpy

    Returns:
        xyz(list[float]): x,y,z coordinates
        rpy(list[float]): roll, pitch, and yaw values
    """
    xyz = matrix[:-1, -1]
    r = R.from_matrix(matrix[:-1, :-1])
    rpy = r.as_rotvec()
    return xyz, rpy


def xyz_rpy_to_matrix4x4(xyz, rpy):
    """Takes xyz coordinates and rpy values and returns a 4x4 transformation matrix created from those values

    Args:
        xyz(list[float]): x,y,z coordinates
        rpy(list[float]): roll, pitch, and yaw values

    Returns:
        matrix: Transformation matrix created from xyz and rpy values
    """
    matrix = np.eye(4)
    r = R.from_rotvec(rpy)
    matrix[:-1, :-1] = r.as_matrix()
    matrix[:-1, -1] = xyz
    return matrix


class AbstractSolver(ABC):
    chain: PyChain

    def __init__(self, chain):
        """Abstract Kinematic Solver class.
        """

    def inverse_solve(self, target_coords, target_rpy, **kwargs):
        """Finds the angles for each joint of the arm given a target end effector.

        Calculates the angles each joint needs to be at given the target end
        effector (x_pos, y_pos, z_pos, roll, pitch, yaw) in cartesion space.

        Args:
            target_coords (list[float]): target end effector XYZ coordinates.
            target_rpy (list[float]): target end effector Roll, Pitch, and Yaw.
            **kwargs:
        Returns:
            angles (list[float]): list of angles for each rotating joint in the chain.
        """

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
