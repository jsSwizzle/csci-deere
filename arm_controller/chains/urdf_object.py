from enum import Enum
from xml.etree.ElementTree import Element
import numpy as np


class URDFMaterial:
    def __init__(self, mat, color, tex):
        self.name = mat.attrib['name']
        if color is not None:
            self.rgba = np.array(color.attrib['rgba'].split()).astype(float)
        if tex is not None:
            self.texture = tex.attrib['filename']

class JointType(Enum):
    """
    Joint Rotation Type
    """
    REVOLUTE = 'revolute'
    CONTINUOUS = 'continuous'
    PRISMATIC = 'prismatic'
    FIXED = 'fixed'
    FLOATING = 'floating'
    PLANAR = 'planar'


class URDFJoint:
    """
    Our representation of URDF Joints
    """

    def __init__(self, joint, parent, child, origin, axis, limit):
        self.name = joint.attrib['name']
        self.type = JointType(joint.attrib['type'])
        self.parent = parent.attrib['link']
        self.child = child.attrib['link']
        self.origin_xyz = np.array(origin.attrib['xyz'].split()).astype(float)
        self.origin_rpy = np.array(origin.attrib['rpy'].split()).astype(float)
        if axis is not None:
            v3 = np.array(axis.attrib['xyz'].split())
            self.axis_xyz = v3
        if limit is not None:
            self.limit_lower = float(limit.attrib['lower'])
            self.limit_upper = float(limit.attrib['upper'])


class GeometryType(Enum):
    """
    Geometry Shape Type
    """
    BOX = 'box'
    CYLINDER = 'cylinder'
    SPHERE = 'sphere'
    MESH = 'mesh'

class URDFVisual:
    def __init__(self, origin, geometry, material):
        self.origin_xyz = np.array(origin.attrib['xyz'].split()).astype(float)
        self.origin_rpy = np.array(origin.attrib['rpy'].split()).astype(float)
        self.geometry_type = GeometryType(geometry[0].tag)
        self.geo_attrib = {}
        for key in geometry[0].attrib:
            self.geo_attrib[key] = geometry[0].attrib[key]
        self.mat_name = material.attrib['name']


class URDFCollision:
    def __init__(self, origin, geometry):
        self.origin_xyz = np.array(origin.attrib['xyz'].split()).astype(float)
        self.origin_rpy = np.array(origin.attrib['rpy'].split()).astype(float)
        self.geometry_type = GeometryType(geometry[0].tag)
        self.geometry_attrib = {}
        for key in geometry[0].attrib:
            self.geometry_attrib[key] = geometry[0].attrib[key]


class URDFLink:
    def __init__(self, link, visuals, collisions):
        self.name = link.attrib['name']
        self.visuals = []
        for viz in visuals:
            self.visuals.append(
                URDFVisual(
                    viz.find('origin'),
                    viz.find('geometry'),
                    viz.find('material')
                )
            )
        self.collisions = []
        for coll in collisions:
            self.visuals.append(
                URDFVisual(
                    coll.find('origin'),
                    coll.find('geometry'),
                    coll.find('material')
                )
            )


class URDFObject:
    def __init__(self, path, mats, links, joints):
        self.path = path
        self.mats = mats
        self.links = links
        self.joints = joints
