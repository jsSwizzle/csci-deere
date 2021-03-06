"""Implementation of Solver Class to solve Kinematics of Arm Class using PyKDL library.
"""
import math
import numpy as np
from PyKDL import *

from arm_controller.chains.py_chain import PyChain
from arm_controller.chains.urdf_object import JointType
from arm_controller.solvers.abstract_solver import AbstractSolver


class PyKDLSolver(AbstractSolver):

    def __init__(self, chain):
        """Basic constructor for Solver class.
        """
        self.chain = chain
        self._kdlChain = Chain()
        joint_mins = JntArray(chain.number_of_joints())
        joint_maxs = JntArray(chain.number_of_joints())
        i = 0
        # for segment in chain.segments:
        for jnt in chain.urdf.joints:
            rotation_frame = Frame(Rotation.EulerZYX(np.rad2deg(jnt.origin_rpy[2]),
                                                     np.rad2deg(jnt.origin_rpy[1]),
                                                     np.rad2deg(jnt.origin_rpy[0])))
            vector_frame = Frame(Vector(jnt.origin_xyz[0], jnt.origin_xyz[1], jnt.origin_xyz[2]))
            frame = rotation_frame * vector_frame

            joint = Joint()
            if jnt.axis_xyz[0] == 1 or jnt.axis_xyz[0] == -1:
                joint = Joint(Joint.RotX)
            elif jnt.axis_xyz[1] == 1 or jnt.axis_xyz[1] == -1:
                joint = Joint(Joint.RotY)
            elif jnt.axis_xyz[2] == 1 or jnt.axis_xyz[2] == -1:
                joint = Joint(Joint.RotZ)
            self._kdlChain.addSegment(Segment(joint, frame))

            if jnt.limit_lower is not None:
                joint_mins[i] = math.radians(jnt.limit_lower)
                if jnt.limit_upper is not None:
                    joint_maxs[i] = math.radians(jnt.limit_upper)
                i += 1

        self._fkSolver = ChainFkSolverPos_recursive(self._kdlChain)
        self._ikSolverVel = ChainIkSolverVel_pinv(self._kdlChain)
        self._ikSolver = ChainIkSolverPos_NR_JL(self._kdlChain,
                                                joint_mins,
                                                joint_maxs,
                                                self._fkSolver,
                                                self._ikSolverVel,
                                                10_000)

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
        initial_angles = kwargs['initial_angles']
        ikJointFinal = JntArray(self._kdlChain.getNrOfJoints())
        ikJointInitial = JntArray(self._kdlChain.getNrOfJoints())
        j = 0
        for angle in initial_angles:
            ikJointInitial[j] = math.radians(angle)
            j += 1
        target_rot = Rotation.RPY(math.radians(target_rpy[0]), math.radians(target_rpy[1]), math.radians(target_rpy[2]))
        target_vec = Vector(target_coords[0], target_coords[1], target_coords[2])
        ikFrameTarget = Frame(target_rot, target_vec)
        ikSuccess = self._ikSolver.CartToJnt(ikJointInitial, ikFrameTarget, ikJointFinal)

        angles = []
        for angle in ikJointFinal:
            angles.append(math.degrees(angle))

        return angles

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
        fkJointInitial = JntArray(self._kdlChain.getNrOfJoints())

        i = 0
        for angle in angles:
            fkJointInitial[i] = math.radians(angle)
            i += 1

        fkEffectorFrame = Frame()
        fkSuccess = self._fkSolver.JntToCart(fkJointInitial, fkEffectorFrame)

        coords = []
        for i in fkEffectorFrame.p:
            coords.append(i)

        rpy = []
        for i in fkEffectorFrame.M.GetRPY():
            rpy.append(math.degrees(i))

        return coords, rpy

    def segmented_forward_solve(self, current_angles):
        """Finds the (x, y, z) position of every joint in the chain (including the end effector).

        Returns:
            coords {list} -- 2 dimensional list containing sets of (X, Y, Z) coordinates of each joint.
        """
        coords = []
        fkJointInitial = JntArray(self._kdlChain.getNrOfJoints())

        i = 0
        for angle in current_angles:
            if angle != None:
                fkJointInitial[i] = math.radians(angle)
                i += 1

        for j in range(0, self._kdlChain.getNrOfSegments()):

            fkEffectorFrame = Frame()
            fkSuccess = self._fkSolver.JntToCart(fkJointInitial, fkEffectorFrame, j)

            intermediate_coords = []
            for i in fkEffectorFrame.p:
                intermediate_coords.append(i)
            coords.append(intermediate_coords)

        coords.append(self.forward_solve(current_angles)[0])
        return coords
