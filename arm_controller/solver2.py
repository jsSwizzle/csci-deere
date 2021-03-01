import numpy as np

from pybotics.kinematic_chain import KinematicChain
from pybotics.link import Link
from pybotics.robot import Robot
from pybotics.predefined_models import ur10

from ikpy import link, chain, inverse_kinematics

# from arm import Arm
from arm_controller.py_chain import PyChain
from arm_controller.py_segment import PySegment

#Solver using Pybotics
class Solver2:

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

#Solver Using IKPy
class Solver3:

    def __init__(self, chain):
        """Basic constructor for Solver class.
        """
        self._rbt = Robot.from_parameters(ur10())
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
    # construct segments for the chain
    world_segment = PySegment('seg0', 'world', 0.0, 0.0, 0.0, [0.0,0.0,0.0], [0.0,0.0,56.0], None)
    waist_segment = PySegment('seg1', 'waist', 90.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,42.93], 'Z', joint_no=0)
    shoulder_segment = PySegment('seg2', 'shoulder', 30.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,120.0], 'Y', joint_no=1)
    elbow_segment = PySegment('seg3', 'elbow', 35.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,118.65], 'Y', joint_no=2)
    wrist_roll_segment = PySegment('seg4', 'wrist_roll', 140.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,60.028], 'Z', joint_no=4)
    wrist_pitch_segment = PySegment('seg5', 'wrist_pitch', 80.0, 0.0, 180.0, [0.0,0.0,0.0], [0.0,0.0,30.17], 'Y', joint_no=5)

    # add the segments to the chain
    myChain = PyChain()
    myChain.append_segment(world_segment)
    myChain.append_segment(waist_segment)
    myChain.append_segment(shoulder_segment)
    myChain.append_segment(elbow_segment)
    myChain.append_segment(wrist_roll_segment)
    myChain.append_segment(wrist_pitch_segment)


    slvr2 = Solver2(myChain)
    # joints = np.deg2rad([5, 5, 5, 5, 5, 5])
    joints = np.deg2rad([item.default_value for item in myChain.segments])
    pose = slvr2._rbt.fk(joints)
    print(pose)
    init_angs = [0,0,0,0,0,0]
    print(f'done! {np.rad2deg(slvr2.inverse_solve(init_angs, pose, [0,0,0]))}')
    slvr3 = Solver3(myChain)

if __name__ == '__main__':
    main()