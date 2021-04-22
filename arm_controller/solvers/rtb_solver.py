"""Implementation of Solver Class to solve Kinematics of Arm Class using IKPy library.
"""

from arm_controller.chains.py_chain import PyChain
import roboticstoolbox as rtb
import roboticstoolbox.robot.ETS as ETS
import numpy as np

from arm_controller.solvers.abstract_solver import AbstractSolver, matrix4x4_to_xyz_rpy, xyz_rpy_to_matrix4x4

"""Location for URDF files
/Lib/site-packages/rtbdata/xacro
"""


class RTBSolver(AbstractSolver):

    def __init__(self, chain: PyChain):
        """Abstract Kinematic Solver class.

        Location for URDF files are relative to:
        /venv/Lib/site-packages/rtbdata/xacro
        """
        self.chain = chain
        links, name = rtb.ERobot.urdf_to_ets_args(self,
                                                  'mechatronics_arm.urdf')
        self._robot = rtb.ERobot(links)
        self._robot.name = name

    def inverse_solve(self, target_coords=[0, 0, 0], target_rpy=[0, 0, 0], **kwargs) -> list[float]:
        """Finds the angles for each joint of the arm given a target end effector.

        Calculates the angles each joint needs to be at given the target end
        effector (x_pos, y_pos, z_pos, roll, pitch, yaw) in cartesion space.

        Args:
            target_coords {list} -- target end effector XYZ coordinates.
            target_rpy {list} -- target end effector Roll, Pitch, and Yaw.
            initial_angles {list} -- initial angle position for each rotating joint in the chain.

        Returns:
            angles {list} -- list of angles for each rotating joint in the chain.
        """
        initial_angles = np.array(kwargs['initial_angles'])
        end_link = kwargs['end_link']
        se3 = xyz_rpy_to_matrix4x4(target_coords, target_rpy)
        ik1 = self._robot.ikine_min(se3, q0=initial_angles)
        move = rtb.jtraj(self._robot.q, ik1.q, 50)
        # self._robot.plot(move.y, backend='pyplot')
        return ik1.q

    def forward_solve(self, angles, **kwargs):
        """Finds the (x, y, z, roll, pitch, yaw) position of the end effector of the chain.

        Calculates the current (x, y, z, roll, pitch, yaw) position of the end
        effector of the arm using the given angles of each of the joints.

        Args:
            current_angles {list} -- list of current angles of each rotating joint in the chain.

        Returns:
            coords {list} -- list containing XYZ coordinates of the end effector.
            rpy {list} -- list containing Roll, Pitch, and Yaw of the end effector.

            :param angles:
            :param **kwargs:
            :keyword end_link: name of end link of chain for calculation
        """
        end_link = kwargs['end_link']
        return matrix4x4_to_xyz_rpy(self._robot.fkine(angles, end_link).A)

    def segmented_forward_solve(self, angles):
        """Finds the (x, y, z) position of every joint in the chain (including the end effector).

        Returns:
            coords {list} -- 2 dimensional list containing sets of (X, Y, Z) coordinates of each joint.
        """
        tuples = []
        for joint in self.chain.urdf.joints:
            self.forward_solve()


def main():
    solver = RTBSolver(None)
    fk = solver.forward_solve(solver._robot.q, end_link='claw_end_link')
    solve = solver.inverse_solve(target_coords=[.08, .08, .09],
                                 target_rpy=[0, 90, 0],
                                 initial_angles=solver._robot.q,
                                 end_link='claw_end_link')
    print(fk)


if __name__ == '__main__':
    main()
