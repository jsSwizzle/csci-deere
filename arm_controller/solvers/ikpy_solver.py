"""Implementation of Solver Class to solve Kinematics of Arm Class using IKPy library.
"""
import numpy as np
from ikpy import chain as ikpc
from arm_controller.chains.py_chain import PyChain
from arm_controller.chains.py_segment import PySegment
from arm_controller.solvers.abstract_solver import AbstractSolver
import matplotlib.pyplot as plt


class IKPySolver(AbstractSolver):

    def __init__(self, chain: PyChain):
        """Basic constructor for Solver class.
        """
        self._chain = ikpc.Chain(ikpc.URDF.get_urdf_parameters(chain.urdf.path,
                                                               [chain.urdf.links[0].name]))

    def inverse_solve(self, target_coords=[0, 0, 0], target_rpy=[0, 0, 0], **kwargs) -> list[float]:
        """
        :param target_coords:
        :param target_rpy:
        :param kwargs:
            :keyword orientation_mode:
        :return: angles_list
        """
        ornt_mode = kwargs['orientation_mode']
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
        return self._chain.forward_kinematics(angles)

    def segmented_forward_solve(self, current_angles):
        """Finds the (x, y, z) position of every joint in the chain (including the end effector).

        Returns:
            coords {list} -- 2 dimensional list containing sets of (X, Y, Z) coordinates of each joint.
        """


if __name__ == '__main__':
    chain = PyChain('../urdf/arm.urdf')
    solver = IKPySolver(chain)
    ikSolv = solver.inverse_solve(target_coords=[.08, .14, .09], target_rpy=[180, 180, 180], orientation_mode='Z')
    fkSolv = solver.forward_solve(ikSolv)
    ax = plt.figure().add_subplot(111, projection='3d')
    solver._chain.plot(ikSolv, ax)
    solver._chain.plot(solver._chain.inverse_kinematics_frame(fkSolv), ax)
    plt.show()
