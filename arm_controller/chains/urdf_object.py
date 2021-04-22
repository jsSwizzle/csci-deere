from enum import Enum
from xml.etree.ElementTree import Element

import numpy as np




class URDFMaterial:
    name: str
    rgba: list[float]
    texture: str

    def __init__(self, mat: Element, color: Element = None, tex: Element = None):
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
    name: str
    type: JointType
    parent: str
    child: str
    origin_xyz: list[float]
    origin_rpy: list[float]

    def __init__(self, joint: Element, parent: Element, child: Element, origin: Element, axis: Element, limit: Element):
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
    origin_xyz: list[float] = [0, 0, 0]
    origin_rpy: list[float] = [0, 0, 0]
    geometry_type: GeometryType
    geo_attrib: dict[str, str]
    mat_name: str

    def __init__(self, origin: Element, geometry: Element, material: Element):
        self.origin_xyz = np.array(origin.attrib['xyz'].split()).astype(float)
        self.origin_rpy = np.array(origin.attrib['rpy'].split()).astype(float)
        self.geometry_type = GeometryType(geometry[0].tag)
        self.geo_attrib = {}
        for key in geometry[0].attrib:
            self.geo_attrib[key] = geometry[0].attrib[key]
        self.mat_name = material.attrib['name']


class URDFCollision:
    origin_xyz: list[float] = [0, 0, 0]
    origin_rpy: list[float] = [0, 0, 0]
    geometry_type: GeometryType
    geometry_attrib: dict[str, str]

    def __init__(self, origin: Element, geometry: Element):
        self.origin_xyz = np.array(origin.attrib['xyz'].split()).astype(float)
        self.origin_rpy = np.array(origin.attrib['rpy'].split()).astype(float)
        self.geometry_type = GeometryType(geometry[0].tag)
        self.geometry_attrib = {}
        for key in geometry[0].attrib:
            self.geometry_attrib[key] = geometry[0].attrib[key]


class URDFLink:
    name: str
    visuals: list[URDFVisual] = []
    collisions: list[URDFCollision] = []

    def __init__(self, link: Element, visuals: list[Element], collisions: list[Element]):
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
    path: str
    mats: dict[URDFMaterial] = {}
    links: list[URDFLink] = []
    joints: list[URDFJoint] = []

    def __init__(self, path: str, mats: dict[URDFMaterial], links: list[URDFLink], joints: list[URDFJoint]):
        self.path = path
        self.mats = mats
        self.links = links
        self.joints = joints

