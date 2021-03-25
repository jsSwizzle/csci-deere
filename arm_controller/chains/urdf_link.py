"""Our representation of URDF Links for URDF-Compliant Solvers
"""
from arm_controller.chains.py_segment import PySegment


class URDFLink:
    def __init__(self, name, orgnxyz, orgnrpy, geotype, geoattr, matname, matclr):
        self.name = name
        self.origin_xyz = orgnxyz
        self.origin_rpy = orgnrpy
        self.geo_type = geotype
        self.geo_attr = []
        for attr in geoattr:
            self.geo_attr.push(attr)
        self.mat_name = matname
        self.mat_clr = matclr
