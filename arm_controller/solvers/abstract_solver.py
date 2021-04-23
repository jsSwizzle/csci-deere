from abc import ABC
import numpy as np
from scipy.spatial.transform import Rotation as R

def matrix4x4_to_xyz_rpy(matrix):
    xyz = matrix[:-1, -1]
    r = R.from_matrix(matrix[:-1, :-1])
    rpy = r.as_rotvec()
    return xyz, rpy


def xyz_rpy_to_matrix4x4(xyz, rpy):
    matrix = np.eye(4)
    r = R.from_rotvec(rpy)
    matrix[:-1, :-1] = r.as_matrix()
    matrix[:-1, -1] = xyz
    return matrix


class AbstractSolver(ABC):
    def __init__(self, chain):
        """Abstract Kinematic Solver class.
        """

    def inverse_solve(self, target_coords, target_rpy, **kwargs):
        """Finds the angles for each joint of the arm given a target end effector.

        Calculates the angles each joint needs to be at given the target end
        effector (x_pos, y_pos, z_pos, roll, pitch, yaw) in cartesion space.

        Args:
            target_coords {list} -- target end effector XYZ coordinates.
            target_rpy {list} -- target end effector Roll, Pitch, and Yaw.
            initial_angles {list} -- initial angle position for each rotating joint in the chain.

        Returns:
            angles {list[float]} -- list of angles for each rotating joint in the chain.
        """

    def forward_solve(self, angles, **kwargs):
        """
        Finds the (x, y, z, roll, pitch, yaw) position of the end effector of the chain.

        Calculates the current (x, y, z, roll, pitch, yaw) position of the end
        effector of the arm using the given angles of each of the joints.
        :param angles: list of current angles of each rotating joint in the chain.
        :param kwargs:
        :returns:
            coords: list containing XYZ coordinates of the end effector.
            rpy: list containing Roll, Pitch, and Yaw of the end effector.

        Args:
            current_angles {list} -- list of current angles of each rotating joint in the chain.

        Returns:
            coords {list} -- list containg XYZ coordinates of the end effector.
            rpy {list} -- list containing Roll, Pitch, and Yaw of the end effector.
        """
