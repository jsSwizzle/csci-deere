"""Contains PyChain class to simulate PyKDL Chain with the use of PySegment's.
"""
from arm_controller.chains.py_urdf import PyURDF, URDFObject


class PyChain():
    urdf: URDFObject = None
    current_values: list[float]

    def __init__(self, urdf_file_path=None):
        """Basic constructor for PyChain class.
        """
        self.segments = []
        if urdf_file_path is not None:
            self.urdf = PyURDF.parse(urdf_file_path)

    def number_of_segments(self):
        """Returns the number of segments within the chain.

        Returns:
            segments {int} -- number of segments in the chain.
        """
        return len(self.segments)

    def number_of_joints(self):
        """Returns the number of rotating joints within the chain.

        Returns:
            joints {int} -- number of rotating joints within the chain.
        """
        joints = 0
        for segment in self.segments:
            if segment.joint_rot != None:
                joints += 1
        return joints

    def push_segment(self, new_segment):
        """Pushes given segment onto the front of the chain.

        Pushes given segment onto the front of the chain, given that the segment.id is unique
        from the other ID's in the rest of the chain.

        Args:
            new_segment {PySegment} -- new PySegment to push to the front of the chain.
        """
        for segment in self.segments:
            if segment.id == new_segment.id:
                print('Segment IDs need to be unique. Unable to add segment to chain.')
                return
        self.segments.insert(0, new_segment)

    def append_segment(self, new_segment):
        """Appends given segment onto the end of the chain.

        Appends given segment onto the end of the chain, given that the segment.id is unique
        from the other ID's in the rest of the chain.

        Args:
            new_segment {PySegment} -- new PySegment to append to the end of the chain.
        """
        for segment in self.segments:
            if segment.id == new_segment.id:
                print('Segment IDs need to be unique. Unable to add segment to chain.')
                return
        self.segments.append(new_segment)

    def get_current_values(self):
        """Gets the current values of each rotating joint in the chain.

        Returns:
            current_values {list} -- list containing each rotating joints current value.
        """
        current_values = []
        for segment in self.segments:
            if segment.joint_rot != None:
                current_values.append(segment.current_val)
        return current_values
