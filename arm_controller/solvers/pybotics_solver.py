"""Implementation of Solver Class to solve Kinematics of Arm Class using PyBotics library.
"""
import numpy as np
from pybotics.robot import Robot
from pybotics.predefined_models import ur10
from arm_controller.chains.py_chain import PyChain
from arm_controller.chains.py_segment import PySegment


class PyBoticsSolver:
    ur10_MDH = [[0, 0, 0, 118],
                [np.pi / 2, 0, np.pi, 0],
                [0, 612.7, 0, 0],
                [0, 571.6, 0, 163.9],
                [-np.pi / 2, 0, 0, 115.7],
                [np.pi / 2, 0, np.pi, 92.2]]
    test_MDH = [[0, 0, 0, 98.507],
                [np.pi / 2, 120, 0, 0],
                [0, 118.65, 0, 0],
                [-np.pi / 2, 0, 0, 60.028],
                [np.pi / 2, 0, 0, 60]
                ]

    def __init__(self, chain):
        """Basic constructor for Solver class.
        """
        self._rbt = Robot.from_parameters(ur10())
        # joint_mins = JntArray(chain.number_of_joints())
        # joint_maxs = JntArray(chain.number_of_joints())
        # i = 0
        # for segment in chain.segments:
        #     rotation_frame = Frame(Rotation.EulerZYX(segment.rotation[0], segment.rotation[1], segment.rotation[2]))
        #     vector_frame = Frame(Vector(segment.translation[0], segment.translation[1], segment.translation[2]))
        #     frame = rotation_frame * vector_frame
        #     joint = None
        #     if segment.joint_rot == None:
        #         joint = Joint()
        #     elif segment.joint_rot == 'X':
        #         joint = Joint(Joint.RotX)
        #     elif segment.joint_rot == 'Y':
        #         joint = Joint(Joint.RotY)
        #     elif segment.joint_rot == 'Z':
        #         joint = Joint(Joint.RotZ)
        #     self._kdlChain.addSegment(Segment(joint, frame))
        #     if segment.joint_rot != None:
        #         joint_mins[i] = math.radians(segment.min_value)
        #         joint_maxs[i] = math.radians(segment.max_value)
        #         i += 1

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
        return self._rbt.ik(target_coords)

    def forward_solve(self, current_angles):
        """Finds the (x, y, z, roll, pitch, yaw) position of the end effector of the chain.

        Calculates the current (x, y, z, roll, pitch, yaw) position of the end
        effector of the arm using the given angles of each of the joints.

        Args:
            current_angles {list} -- list of current angles of each rotating joint in the chain.

        Returns:
            coords {list} -- list containing XYZ coordinates of the end effector.
            rpy {list} -- list containing Roll, Pitch, and Yaw of the end effector.
        """

    def segmented_forward_solve(self, current_angles):
        """Finds the (x, y, z) position of every joint in the chain (including the end effector).

        Returns:
            coords {list} -- 2 dimensional list containing sets of (X, Y, Z) coordinates of each joint.
        """


def main():
    pbSolv = PyBoticsSolver(PyChain())
    print(pbSolv.inverse_solve([0,90,150,10,90,80,0], [200, 120, 90], [0, 0, 0]))


if __name__ == '__main__':
    main()
