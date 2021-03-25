from arm_controller.chains.py_segment import PySegment
import numpy as np


class URDFJoint:
    """
    Our representation of URDF Joints for URDF-Compliant Solvers
    """

    def __init__(self, ps: PySegment):
        if ps.joint_rot == 'X':
            axis = [1, 0, 0]
        elif ps.joint_rot == 'Y':
            axis = [0, 1, 0]
        elif ps.joint_rot == 'Z':
            axis = [0, 0, 1]
        else:
            axis = [0, 0, 0]
        self.name = ps.function
        self.origin_xyz = ps.translation
        self.origin_rpy = ps.rotation
        self.axis_xyz = axis
        self.limit_lower = np.deg2rad(ps.min_value)
        self.limit_upper = np.deg2rad(ps.max_value)
