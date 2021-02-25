"""Implementation of Solver Class to solve Kinematics of Arm Class.
"""
from .py_chain import PyChain
from .py_segment import PySegment
from PyKDL import *
import math

class Solver:

    def __init__(self, chain):
        """Basic constructor for Solver class.
        """
        self._kdlChain = Chain()
        for segment in chain.segments:
            rotation_frame = Frame(Rotation.EulerZYX(segment.rotation[0], segment.rotation[1], segment.rotation[2]))
            vector_frame = Frame(Vector(segment.vector[0], segment.vector[1], segment.vector[2]))
            frame = rotation_frame * vector_frame
            joint = None
            if segment.joint_rot == None:
                joint = Joint()
            elif segment.joint_rot == 'X':
                joint = Joint(Joint.RotX)
            elif segment.joint_rot == 'Y':
                joint = Joint(Joint.RotY)
            elif segment.joint_rot == 'Z':
                joint = Joint(Joint.RotZ)
            self._kdlChain.addSegment(Segment(joint, frame))

        self._fkSolver = ChainFkSolverPos_recursive(self._kdlChain)
        self._ikSolverVel = ChainIkSolverVel_pinv(self._kdlChain)
        self._ikSolver = ChainIkSolverPos_NR(self._kdlChain, self._fkSolver, self._ikSolverVel)

    def inverse_solve(self, initial_angles, target_coords, target_rpy):
        """Finds the angles for each joint of the arm given a target end effector.

        Calculates and the angles each joint needs to be at given the target end
        effector (x_pos, y_pos, z_pos, roll, pitch, yaw) in cartesion space.

        Args:
            initial_angles {list} -- initial angle position for each joint in the chain.
            target_coords {list} -- target end effector XYZ coordinates.
            target_rpy {list} -- target end effector Roll, Pitch, Yaw.

        Returns:
            angles {list} -- list of angles for each servo in the arm.
        """
        ikJointInitial = JntArray(self._kdlChain.getNrOfJoints())
        i = 0
        for angle in initial_angles:
            ikJointInitial[i] = math.radians(angle)
            i += 1

        ikJointFinal = JntArray(self._kdlChain.getNrOfJoints())
        ikFrameTarget = Frame(Rotation.RPY(math.radians(target_rpy[0]), math.radians(target_rpy[1]), math.radians(target_rpy[2])), Vector(target_coords[0], target_coords[1], target_coords[2]))
        print(ikFrameTarget)
        ikSuccess = self._ikSolver.CartToJnt(ikJointInitial, ikFrameTarget, ikJointFinal)

        angles = []
        for angle in ikJointFinal:
            angles.append(math.degrees(angle))

        return angles


    def forward_solve(self, current_angles):
        """Finds the (x, y, z) position of the end effector of the arm.

        Calculates the current (x, y, z) position of the end effector of the arm
        using the given angles of each of the joints.

        Args:
            current_angles {list} -- list of current angles of each segment in the chain.

        Returns:
            coords {list} -- list containing XYZ coordinates of the end effector.
            rpy {list} -- list containing Roll, Pitch, and Yaw of the end effector.
        """
        fkJointInitial = JntArray(self._kdlChain.getNrOfJoints())

        i = 0
        for angle in current_angles:
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
