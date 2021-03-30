"""URDF parser / static methods
"""
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element
from ikpy import chain as ipc
import matplotlib.pyplot as plt

from arm_controller.urdf.urdf_joint import URDFJoint
from arm_controller.urdf.urdf_link import URDFLink
from arm_controller.urdf.urdf_material import URDFMaterial


class PyURDF:
    def parse(filepath):
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
                               item.findall('visual'))
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
        for m in mats:
            print(f'{m.name}')
        for l in links:
            print(l.name)
        for j in joints:
            print(j.name)

    def testParse(filepath, baseLink):
        """test parse that uses IKPy URDF parser to confirm against / validate our parser
        """
        chain = ipc.Chain(ipc.URDF.get_urdf_parameters(filepath, [baseLink]))
        print(chain)
        jointList = []
        # defaultVals = [0, 90, 30, 10, 90, 80, 0]
        # jointList.append([np.deg2rad(item) for item in defaultVals])
        jointList.append(chain.inverse_kinematics([-.08, .06, .090], [0, 0, 0], 'X'))
        jointList.append(chain.inverse_kinematics([.08, .04, .060], [0, 0, 0], 'Z'))
        jointList.append(chain.inverse_kinematics([-.03, .18, .190], [0, 0, 0]))
        jointList.append(chain.inverse_kinematics([-.12, .08, .190], [0, 0, 90], 'X'))
        jointList.append(chain.inverse_kinematics([.12, .08, .190], [0, 0, 90], 'Z'))
        ax = plt.figure().add_subplot(111, projection='3d')
        for jnts in jointList:
            chain.plot(jnts, ax)
        plt.show()


if __name__ == '__main__':
    # PyURDF.testParse('arm.urdf', 'world_base_link')
    # PyURDF.testParse('armageddon.urdf', 'world_base_link')
    # PyURDF.testParse('test.urdf', 'link1')
    PyURDF.parse('arm.urdf')
