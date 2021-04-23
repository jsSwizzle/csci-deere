"""Implementation of Solver Class to solve Kinematics of Arm Class using IKPy library.
"""

from arm_controller.chains.py_chain import PyChain
import roboticstoolbox as rtb
import roboticstoolbox.robot.ETS as ETS
import numpy as np
import os

from arm_controller.solvers.abstract_solver import AbstractSolver, matrix4x4_to_xyz_rpy, xyz_rpy_to_matrix4x4

"""Location for URDF files
/Lib/site-packages/rtbdata/xacro
"""


class RTBSolver(AbstractSolver):

    def __init__(self, chain: PyChain):
        """Abstract Kinematic Solver class.

        Location for URDF files are relative to:
        /Lib/site-packages/rtbdata/xacro
        """
        self.chain = chain
        # dirname = os.path.dirname(__file__)
        # filepath = os.path.join(dirname, chain.urdf.path)
        links, name = rtb.ERobot.urdf_to_ets_args(self,
                                                  'mechatronics_arm.urdf')
        self._robot = rtb.ERobot(links)
        self._robot.name = name

    def inverse_solve(self, target_coords, target_rpy, **kwargs):
        """Finds the angles for each joint of the arm given a target end effector.

        Calculates the angles each joint needs to be at given the target end
        effector (x_pos, y_pos, z_pos, roll, pitch, yaw) in cartesion space.

        Args:
            target_coords (list[float]): target end effector XYZ coordinates.
            target_rpy (list): target end effector Roll, Pitch, and Yaw.
            **kwargs:
                initial_angles (list[float]): initial angles to solve from |
                end_link (str): end link name to solve for

        Returns:
            angles (list): list of angles for each rotating joint in the chain.
        """
        if 'initial angles' in kwargs:
            initial_angles = np.array(kwargs['initial_angles']).astype(float)
        else:
            initial_angles = self._robot.q
        if 'end_link' in kwargs:
            end_link = kwargs['end_link']
        else:
            end_link = self._robot.ee_links[0].name

        se3 = xyz_rpy_to_matrix4x4(target_coords, target_rpy)
        ik1 = self._robot.ikine_min(se3, q0=initial_angles)
        # ik1 = self._robot.ikine_LMS(se3, q0=initial_angles, ilimit=5000)
        # move = rtb.jtraj(self._robot.q, ik1.q, 50)
        # self._robot.plot(move.y, backend='pyplot')
        return ik1.q

    def forward_solve(self, angles, **kwargs):
        """Finds the (x, y, z, roll, pitch, yaw) position of the end effector of the chain.

        Calculates the current (x, y, z, roll, pitch, yaw) position of the end
        effector of the arm using the given angles of each of the joints.

        Args:
            angles (list): list of current angles of each rotating joint in the chain.
            **kwargs:
                end_link: name of end effector to calculate
        Returns:
            coords (list): list containing XYZ coordinates of the end effector.
            rpy (list): list containing Roll, Pitch, and Yaw of the end effector.
        """
        if 'end_link' in kwargs:
            end_link = kwargs['end_link']
        else:
            end_link = self._robot.ee_links[0].name
        return matrix4x4_to_xyz_rpy(self._robot.fkine(angles, end_link).A)

    def segmented_forward_solve(self, angles):
        """Finds the (x, y, z) position of every joint in the chain (including the end effector).

        Returns:
            coords (list): 2 dimensional list containing sets of (X, Y, Z) coordinates of each joint.
        """
        tuples = []
        for link in self.chain.urdf.links:
            tuples.append(self.forward_solve(angles=angles, end_link=link.name)[0])
        return tuples


if __name__ == '__main__':
    chain = PyChain(urdf_file_path='../urdf/mechatronics_arm.urdf')
    solver = RTBSolver(chain)
    ik = solver.inverse_solve(target_coords=[.06, .06, .09], target_rpy=[0, 135, 0])
    fk = solver.forward_solve(angles=ik)
    sfk = solver.segmented_forward_solve(angles=ik)
    print(solver)
