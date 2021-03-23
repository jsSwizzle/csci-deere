from arm_controller.chains.py_chain import PyChain
from arm_controller.chains.py_segment import PySegment
import xml.etree.ElementTree as ET

"""
    URDF parser / static methods
"""
class PyURDF:
    def parse(self, filepath):
        tree = ET.parse(source=filepath)
        root = tree.getroot()
