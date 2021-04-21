"""URDF parser / static methods
"""
import xml.etree.ElementTree as ET

from arm_controller.urdf.urdf_joint import URDFJoint
from arm_controller.urdf.urdf_link import URDFLink
from arm_controller.urdf.urdf_material import URDFMaterial
from arm_controller.urdf.urdf_object import URDFObject


class PyURDF:

    def parse(filepath) -> URDFObject:
        tree = ET.parse(source=filepath)
        root = tree.getroot()
        s = ''
        mats: list[URDFMaterial] = []
        links: list[URDFLink] = []
        joints: list[URDFJoint] = []
        for item in root:
            tag = item.tag
            if tag == 'material':
                mat: URDFMaterial = URDFMaterial(item,
                                                 item.find('color'))
                mats.append(mat)
                continue
            elif tag == 'link':
                lnk = URDFLink(item,
                               item.findall('visual'),
                               item.findall('collision'))
                links.append(lnk)
                continue
            elif tag == 'joint':
                s += f'{tag} '
                jnt = URDFJoint(item,
                                item.find('parent'),
                                item.find('child'),
                                item.find('origin'),
                                item.find('axis'),
                                item.find('limit'))
                joints.append(jnt)

        return URDFObject(filepath, mats, links, joints)


if __name__ == '__main__':
    urdf = PyURDF.parse('mechatronics_arm.urdf')
