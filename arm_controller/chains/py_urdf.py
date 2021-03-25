"""URDF parser / static methods
"""
from arm_controller.chains.py_chain import PyChain
from arm_controller.chains.py_segment import PySegment
import xml.etree.ElementTree as ET
from ikpy import link as ipl, chain as ipc
import numpy as np
import matplotlib.pyplot as plt


class PyURDF:
    def parse(filepath):
        tree = ET.parse(source=filepath)
        root = tree.getroot()
        s = ''
        for item in root:
            s += f'{item.tag} '
        print(s)

    def testParse(filepath, baseLink):
        """test parse that uses IKPy URDF parser to confirm against / validate our parser
        """
        chain = ipc.Chain(ipc.URDF.get_urdf_parameters(filepath, [baseLink]))
        print(chain)
        defaultVals = [0, 90, 30, 10, 90, 80, 0]
        joints0 = [np.deg2rad(item) for item in defaultVals]
        joints1 = chain.inverse_kinematics([-.08, .06, .090], [0, 0, 0], 'X')
        joints2 = chain.inverse_kinematics([-.08, .06, .090], [0, 0, 0], 'Z')
        joints3 = chain.inverse_kinematics([-.12, .08, .190], [0, 0, 0])
        joints4 = chain.inverse_kinematics([-.12, .08, .190], [0, 0, 90], 'Z')
        ax = plt.figure().add_subplot(111, projection='3d')
        chain.plot(joints0, ax)
        chain.plot(joints1, ax)
        chain.plot(joints2, ax)
        chain.plot(joints3, ax)
        chain.plot(joints4, ax)
        plt.show()


if __name__ == '__main__':
    PyURDF.testParse('arm.urdf', 'world_base_link')
    PyURDF.parse('arm.urdf')
    # PyURDF.testParse('test.urdf', 'link1')
