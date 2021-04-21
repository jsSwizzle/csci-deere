"""Our representation of URDF Links for URDF-Compliant Solvers
"""
from enum import Enum
from xml.etree.ElementTree import Element

from arm_controller.chains.py_segment import PySegment


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
        self.origin_xyz = origin.attrib['xyz']
        self.origin_rpy = origin.attrib['rpy']
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
        self.origin_xyz = origin.attrib['xyz']
        self.origin_rpy = origin.attrib['rpy']
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
