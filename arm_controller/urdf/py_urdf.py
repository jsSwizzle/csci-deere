"""URDF parser / static methods
"""
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element

from arm_controller.urdf.urdf_joint import URDFJoint
from arm_controller.urdf.urdf_link import URDFLink
from arm_controller.urdf.urdf_material import URDFMaterial
from arm_controller.urdf.urdf_object import URDFObject


class PyURDF:

    def parse(filepath) -> URDFObject:
        """
        Parses a URDF file into a URDFObject, a pythonic representation of URDF
        :return:
        """
        tree = ET.parse(source=filepath)
        root = tree.getroot()
        s = ''
        links: list[URDFLink] = []
        joints: list[URDFJoint] = []
        materials: dict[URDFMaterial] = {}
        # This gathers all instances of materials and adds them to a dictionary, assuring that all materials are defined
        # and referenceable by name even if they are defined within links
        mat_iter = root.iter('material')
        all_mats = [m for m in mat_iter]
        for mat in all_mats:
            if mat.find('color') is not None or mat.find('texture') is not None:
                mat_name = mat.attrib['name']
                if mat_name not in materials:
                    materials[mat_name] = URDFMaterial(mat,
                                                       mat.find('color'),
                                                       mat.find('texture'))
                else:
                    print(f'material already exists: {mat_name}')
        for item in root:
            tag = item.tag
            if tag == 'material':
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

        return URDFObject(filepath, materials, links, joints)


if __name__ == '__main__':
    # urdf = PyURDF.parse('mechatronics_arm.urdf')
    urdf = PyURDF.parse('armageddon.urdf')
