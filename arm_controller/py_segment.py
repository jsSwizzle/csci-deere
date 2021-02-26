"""Implementation of a segment class that acts as a wrapper for data that defines a segment.
"""

class PySegment():
    def __init__(self, segment_id, function, default, min_value, max_value, rotation, translation, joint_rot, current_val=0.0, joint_no=-1):
        """Basic constructor for PySegment.

        Args:
            segment_id {str} -- unique segment id to identify the segment from others in the same chain.
            function {str} -- user defined function of the segment (e.g. the 'shoulder' segment).
            default {float} -- default value the joint is set to by default.
            min_value {float} -- minimum value the joint is allowed to rotate too.
            max_value {float} -- maximum value the joint is allowed to rotate too.
            rotation {list} -- list of floats defining the rotation of the segmens frame, given as rotations around ZYX respectively.
            translation {list} -- list of floats defining the translation of the segments frame in 3 cartesian dimensions.
            joint_rot {str} -- defines the axis for which the joint rotates around (acceptable values None, 'X', 'Y', 'Z').
            current_val {float} -- holds the current value of the joint for the given segment (default 0.0).
            joint_no {int} -- joint number for purposes of setting your own joint order for ServoKit (default -1).
        """
        self.id = segment_id
        self.function = function
        self.default_value = default
        self.min_value = min_value
        self.max_value = max_value
        self.current_val = current_val
        self.joint_no = joint_no
        self.rotation = rotation
        self.translation = translation
        if joint_rot == None or joint_rot == 'X' or joint_rot == 'Y' or joint_rot == 'Z':
            self.joint_rot = joint_rot
        else:
            self.joint_rot = None
