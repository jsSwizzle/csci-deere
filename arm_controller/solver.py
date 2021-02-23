"""Implementation of Solver Class to solve Kinematics of Arm Class.
"""
from PyKDL import *
import math

class Solver:

    def __init__(self, segment_info, joint_info):
        """Basic constructor for Solver class.
        """
        chain = Chain()
        for segmentID in segment_info:
            pass # TODO: create segments for the chain from given info
        self._kdlChain = chain
        self._fkSolver = ChainFkSolverPos_recursive(chain)
        self._ikSolverVel = ChainIkSolverVel_pinv(chain)
        self._ikSolver = ChainIkSolverPos_NR(chain, self._fkSolver, self._ikSolverVel)

    def inverse_solve(self, initial_angles, x_pos, y_pos, z_pos, roll, pitch, yaw):
        """Finds the angles for each joint of the arm given a target end effector.

        Calculates and the angles each joint needs to be at given the target end
        effector (x_pos, y_pos, z_pos, roll, pitch, yaw) in cartesion space.

        Args:
            initial_angles {dict} -- initial angle position for each joint in the chain (key = jointID, value = current angle).
            x_pos {float} -- target x position for the end effector in cartesian space.
            y_pos {float} -- target y position for the end effector in cartesian space.
            z_pos {float} -- target z position for the end effector in cartesian space.
            roll {float} -- end roll angle for the end effector frame.
            pitch {float} -- end pitch angle for the end effector frame.
            yaw {float} -- end pitch angle for the end effector frame.

        Returns:
            angles {dict} -- list of angles for each servo in the arm (angles[<servoID>]['final_angle'])
        """
        ikJointInitial = JntArray(self._kdlChain.getNrOfJoints())
        i = 0
        for joint in initial_angles:
            if i < self._kdlChain.getNrOfJoints():
                ikJointInitial[i] = math.radians(initial_angles[joint])
            i += 1
        ikJointFinal = JntArray(self._kdlChain.getNrOfJoints)
        ikFrameTarget = Frame(Rotation.RPY(math.radians(roll), math.radians(pitch), math.radians(yaw)), Vector(x_pos, y_pos, z_pos))
        ikSuccess = self._ikSolver.CartToJnt(ikJointInitial, ikFrameTarget, ikJointFinal)
        # TODO; determine proper return value format

    def forward_solve(self, current_angles):
        """Finds the (x, y, z) position of the end effector of the arm.

        Calculates the current (x, y, z) position of the end effector of the arm
        using the given angles of each of the joints.

        Args:
            current_angles {dict} -- dictionary list of current angles for each servo (key = jointID, value = current angle).

        Returns:
            (x_pos, y_pos, z_pos) {list} -- list of individual elements for each coordinate in the cartesian plane
        """
        fkJointInitial = JntArray(self._kdlChain.getNrOfJoints())
        i = 0
        for joint in current_angles:
            if i < self._kdlChain.getNrOfJoints():
                ikJointInitial[i] = math.radians(current_angles[joint])
            i += 1
        fkEffectorFrame = Frame()
        fkSuccess = self._fkSolver.JntToCart(fkJointInitial, fkEffectorFrame)
        # TODO: determine proper return value format
