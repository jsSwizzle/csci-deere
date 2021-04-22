from enum import Enum
from xml.etree.ElementTree import Element
import numpy as np


class URDFMaterial:
    """
    Object representation of a URDF Material
    """

    def __init__(self, mat, color, tex):
        self.name = mat.attrib['name']
        if color is not None:
            self.rgba = np.array(color.attrib['rgba'].split()).astype(float)
        else:
            self.rgba = None
        if tex is not None:
            self.texture = tex.attrib['filename']
        else:
            self.texture = None


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
    Object representation of a URDF Joint
    """

    def __init__(self, joint, parent, child, origin, axis=None, limit=None):
        self.name = joint.attrib['name']
        self.type = JointType(joint.attrib['type'])
        self.parent = parent.attrib['link']
        self.child = child.attrib['link']
        self.origin_xyz = np.array(origin.attrib['xyz'].split()).astype(float)
        self.origin_rpy = np.array(origin.attrib['rpy'].split()).astype(float)
        if axis is not None:
            self.axis_xyz = np.array(axis.attrib['xyz'].split())
        else:
            self.axis_xyz = None
        if limit is not None:
            if 'lower' in limit.attrib:
                self.limit_lower = float(limit.attrib['lower'])
            else:
                self.limit_lower = None
            if 'upper' in limit.attrib:
                self.limit_upper = float(limit.attrib['upper'])
            else:
                self.limit_upper = None
            if 'velocity' in limit.attrib:
                self.limit_velocity = float(limit.attrib['velocity'])
            else:
                self.limit_velocity = None
            if 'effort' in limit.attrib:
                self.limit_effort = float(limit.attrib['effort'])
            else:
                self.limit_effort = None
        else:
            self.limit_lower = None
            self.limit_upper = None
            self.limit_velocity = None
            self.limit_effort = None


class GeometryType(Enum):
    """
    Geometry Shape Type
    """
    BOX = 'box'
    CYLINDER = 'cylinder'
    SPHERE = 'sphere'
    MESH = 'mesh'


class URDFVisual:
    """
    Object representation of a URDF Visual
    """

    def __init__(self, origin, geometry, material):
        self.origin_xyz = np.array(origin.attrib['xyz'].split()).astype(float)
        self.origin_rpy = np.array(origin.attrib['rpy'].split()).astype(float)
        self.geometry_type = GeometryType(geometry[0].tag)
        self.geometry_attrib = {}
        for key in geometry[0].attrib:
            if key == 'size':
                self.geometry_attrib[key] = np.array(geometry[0].attrib[key].split()).astype(float)
            elif key == 'filename':
                self.geometry_attrib[key] = geometry[0].attrib[key]
            else:
                self.geometry_attrib[key] = float(geometry[0].attrib[key])

        self.mat_name = material.attrib['name']


class URDFCollision:
    """
    Object representation of a URDF Collision
    """

    def __init__(self, origin, geometry):
        self.origin_xyz = np.array(origin.attrib['xyz'].split()).astype(float)
        self.origin_rpy = np.array(origin.attrib['rpy'].split()).astype(float)
        self.geometry_type = GeometryType(geometry[0].tag)
        self.geometry_attrib = {}
        for key in geometry[0].attrib:
            if key == 'size':
                self.geometry_attrib[key] = np.array(geometry[0].attrib[key].split()).astype(float)
            elif key == 'filename':
                self.geometry_attrib[key] = geometry[0].attrib[key]
            else:
                self.geometry_attrib[key] = float(geometry[0].attrib[key])


class URDFLink:
    """
    Object representation of a URDF Link
    """

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
    """
    Object representation of a URDF file's elements
    """

    def __init__(self, name, path, mats, links, joints):
        self.name = name
        self.path = path
        self.mats = mats
        self.links = links
        self.joints = joints
