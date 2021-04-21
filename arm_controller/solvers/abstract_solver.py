from abc import ABC

from arm_controller.chains.py_chain import PyChain


class AbstractSolver(ABC):
    def __init__(self, chain: PyChain):
        """Abstract Kinematic Solver class.
        """

    def inverse_solve(self, target_coords=[0, 0, 0], target_rpy=[0, 0, 0], **kwargs) -> list[float]:
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
            current_angles {list} --

        Returns:

        """

