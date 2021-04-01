from xml.etree.ElementTree import Element
from arm_controller.chains.py_segment import PySegment
import numpy as np
from enum import Enum


class JointType(Enum):
    REVOLUTE = 'revolute'
    CONTINUOUS = 'continuous'
    PRISMATIC = 'prismatic'
    FIXED = 'fixed'
    FLOATING = 'floating'
    PLANAR = 'planar'


class URDFJoint:
    """
    Our representation of URDF Joints for URDF-Compliant Solvers
    """
    name: str
    type: JointType
    parent: str
    child: str
    origin_xyz: list[float]
    origin_rpy: list[float]

    # def __init__(self, ps: PySegment):
    #     if ps.joint_rot == 'X':
    #         axis = [1, 0, 0]
    #     elif ps.joint_rot == 'Y':
    #         axis = [0, 1, 0]
    #     elif ps.joint_rot == 'Z':
    #         axis = [0, 0, 1]
    #     else:
    #         axis = [0, 0, 0]
    #     self.name = ps.function
    #     self.origin_xyz = ps.translation
    #     self.origin_rpy = ps.rotation
    #     self.axis_xyz = axis
    #     self.limit_lower = np.deg2rad(ps.min_value)
    #     self.limit_upper = np.deg2rad(ps.max_value)

    def __init__(self, joint: Element, parent: Element, child: Element, origin: Element, axis: Element, limit: Element):
        self.name = joint.attrib['name']
        self.type = JointType(joint.attrib['type'])
        self.parent = parent.attrib['link']
        self.child = child.attrib['link']
        self.origin_xyz: [] = origin.attrib['xyz'].split()
        self.origin_rpy = origin.attrib['rpy'].split()
        if axis is not None:
            self.axis_xyz = axis.attrib['xyz'].split()
        if limit is not None:
            self.limit_lower = limit.attrib['lower']
            self.limit_upper = limit.attrib['upper']
