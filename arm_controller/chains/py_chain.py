"""Contains PyChain class to simulate PyKDL Chain with the use of PySegment's.
"""
from arm_controller.chains.py_urdf import PyURDF, URDFObject


class PyChain():
    def __init__(self, urdf_file_path):
        """Basic constructor for PyChain class.
        """
        self.segments = []
        if urdf_file_path is not None:
            self.urdf = PyURDF.parse(urdf_file_path)

    def number_of_segments(self):
        """Returns the number of segments within the chain.

        Returns:
            segments (int): number of segments in the chain.
        """
        return len(self.segments)

    def number_of_joints(self):
        """Returns the number of rotating joints within the chain.

        Returns:
            joints (int): number of rotating joints within the chain.
        """
        joints = 0
        for segment in self.segments:
            if segment.joint_rot != None:
                joints += 1
        return joints

    def get_current_values(self):
        """Gets the current values of each rotating joint in the chain.

        Returns:
            current_values (list): list containing each rotating joints current value.
        """
        current_values = []
        for segment in self.segments:
            if segment.joint_rot != None:
                current_values.append(segment.current_val)
        return current_values
