from arm_controller.urdf.urdf_joint import URDFJoint
from arm_controller.urdf.urdf_link import URDFLink
from arm_controller.urdf.urdf_material import URDFMaterial

class URDFObject:
    path: str
    mats: list[URDFMaterial] = []
    links: list[URDFLink] = []
    joints: list[URDFJoint] = []

    def __init__(self, path, mats, links, joints):
        self.path = path
        self.mats = mats
        self.links = links
        self.joints = joints