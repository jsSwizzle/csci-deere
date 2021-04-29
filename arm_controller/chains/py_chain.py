"""Contains PyChain class to simulate PyKDL Chain with the use of PySegment's.
"""
import math
from arm_controller.chains.py_urdf import PyURDF, URDFObject

class PyChain():
    def __init__(self, urdf_file_path):
        """Basic constructor for PyChain class.
        """
        self.urdf = PyURDF.parse(urdf_file_path)
        self.joints = {}
        for joint in self.urdf.joints:
                self.joints[joint.name] = {'current_value': 0.0}


    def number_of_segments(self):
        """Returns the number of segments within the chain.
        Args:
            urdf_file_path {str} -- path to the urdf file to use to parse out the URDF objects.
        """
        self.urdf = PyURDF.parse(urdf_file_path)
        self.joints = {}
        for joint in self.urdf.joints:
            self.joints[joint.name] = {'current_value': 0.0}
        self.set_default_values()

    def number_of_joints(self):
        """Returns the number of rotating joints within the chain.

        Returns:
            joints (int): number of rotating joints within the chain.
        """
        return len(self.joints)

    def get_current_values(self):
        """Gets the current values of each rotating joint in the chain.

        Returns:
            current_values {list} -- list containing each rotating joints current value (in degrees).
        """
        current_values = []
        for joint in self.joints:
            current_values.append(self.joints[joint]['current_value'])
        return current_values

    def set_default_values(self, defaults={'waist':90.0,'shoulder':150.0,'elbow':35.0,'wrist_roll':140.0,'wrist_pitch':85.0,'claw':None}):
        """Set the default values for the rotatable joints in the chain.

        Args:
            defaults {dict} -- mapping from the joint names (same as urdf) to their default values (if joint doesn't move use None).
        """
        for joint in defaults:
            if defaults[joint] != None:
                self.joints[joint]['default_value'] = math.radians(defaults[joint])
            else:
                self.joints[joint]['default_value'] = None

    def get_default_values(self):
        """Gets the default values of each rotating joint in the chain.

        Returns:
            default_values {list} -- list containg each rotating joints default value (in degrees).
        """
        default_values = []
        for joint in self.joints:
            default_values.append(self.joints[joint]['default_value'])
        return default_values
