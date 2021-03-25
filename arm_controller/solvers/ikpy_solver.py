"""Implementation of Solver Class to solve Kinematics of Arm Class using IKPy library.
"""
import numpy as np
from ikpy import link as ipl, chain as ipc, inverse_kinematics as ipik
from arm_controller.chains.py_chain import PyChain
from arm_controller.chains.py_segment import PySegment


class IKPySolver:

    def __init__(self, chain: PyChain):
        """Basic constructor for Solver class.
        """
        # links = [ipl.OriginLink()]
        links = []
        for i, s in enumerate(chain.segments):
            axis = []
            if s.joint_rot == 'X':
                axis = [1, 0, 0]
            elif s.joint_rot == 'Y':
                axis = [0, 1, 0]
            elif s.joint_rot == 'Z':
                axis = [0, 0, 1]
            else:
                axis = [0, 0, 0]
            link = ipl.URDFLink(
                name=s.function,
                translation_vector=s.translation,
                orientation=s.rotation,
                rotation=axis,
                bounds=(s.min_value, s.max_value)
            )
            links.append(link)
        self._chain = ipc.Chain(links)

    def inverse_solve(self, initial_angles, target_coords, target_rpy):
        """Finds the angles for each joint of the arm given a target end effector.

        Calculates the angles each joint needs to be at given the target end
        effector (x_pos, y_pos, z_pos, roll, pitch, yaw) in cartesion space.

        Args:
            initial_angles {list} -- initial angle position for each rotating joint in the chain.
            target_coords {list} -- target end effector XYZ coordinates.
            target_rpy {list} -- target end effector Roll, Pitch, and Yaw.

        Returns:
            angles {list} -- list of angles for each rotating joint in the chain.
        """
        return self._chain.inverse_kinematics(target_coords, target_rpy)

    def forward_solve(self, joints):
        """Finds the (x, y, z, roll, pitch, yaw) position of the end effector of the chain.

        Calculates the current (x, y, z, roll, pitch, yaw) position of the end
        effector of the arm using the given angles of each of the joints.

        Args:
            current_angles {list} -- list of current angles of each rotating joint in the chain.

        Returns:
            coords {list} -- list containing XYZ coordinates of the end effector.
            rpy {list} -- list containing Roll, Pitch, and Yaw of the end effector.
        """
        return self._chain.forward_kinematics(joints)

    def segmented_forward_solve(self, current_angles):
        """Finds the (x, y, z) position of every joint in the chain (including the end effector).

        Returns:
            coords {list} -- 2 dimensional list containing sets of (X, Y, Z) coordinates of each joint.
        """
