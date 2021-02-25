
class PySegment():
    def __init__(self, segment_id, function, default, min_value, max_value, rotation, vector, static, current_val=0.0, joint_no=0):
        self.id = segment_id
        # default values and function
        self.function = function
        self.default_value = default
        self.min_value = min
        self.max_value = max
        # current values
        self.current_val = current_val
        self.joint_no = joint_no
        # frame
        self.rotation = rotation
        self.vector = vector
        # joint
        self.is_static = static
