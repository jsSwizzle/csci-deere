from .py_segment import PySegment

class PyChain():
    def __init__(self):
        self.head = None
        self.segments = []

    def number_of_segments(self):
        return len(self.segments)

    def number_of_joints(self):
        count = 0
        for segment in self.segments:
            if segment.joint_rot != None:
                count += 1
        return count

    def push_segment(self, new_segment):
        for segment in self.segments:
            if segment.id == new_segment.id:
                print('Segment IDs need to be unique. Unable to add segment to chain.')
                return
        self.segments.insert(0, new_segment)

    def append_segment(self, new_segment):
        for segment in self.segments:
            if segment.id == new_segment.id:
                print('Segment IDs need to be unique. Unable to add segment to chain.')
                return
        self.segments.append(new_segment)

    def get_current_values(self):
        current_values = []
        for segment in self.segments:
            if segment.joint_rot != None:
                current_values.append(segment.current_val)
        return current_values
