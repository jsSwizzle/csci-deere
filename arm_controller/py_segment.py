
class PySegment():
    def __init__(self, segment_id, function, default, min_value, max_value, rotation, vector, joint_rot, current_val=0.0, joint_no=0):
        self.id = segment_id
        # default values and function
        self.function = function
        self.default_value = default
        self.min_value = min_value
        self.max_value = max_value
        # current values
        self.current_val = current_val
        self.joint_no = joint_no
        # frame
        self.rotation = rotation # [0, 0, pi / 2]
        self.vector = vector # [0, 2, 0]
        # joint
        self.joint_rot = joint_rot
