import math

class Solver:

    def __init__(self, servo_info, segment_info):
        """Basic constructor for Solver class.
        """
        max_range = 0
        for segment in segment_info:
            max_range += segment_info[segment]['segment_length']

        self._max_range = max_range
        self._servo_info = servo_info
        self._segment_info = segment_info


    def _convert_to_spherical(self, x_pos, y_pos, z_pos):
        r = x_pos ** 2 + y_pos ** 2 + z_pos ** 2
        r = math.sqrt(r)

        theta = z / r
        theta = math.acos(theta)

        phi = y / z
        phi = math.atan(phi)

        return r, theta, phi

    def _solve_shoulder_angle(self, theta, l_one, l_two, r):
        a = (l_one ** 2 + r ** 2 - l_two ** 2) / (2 * l_one * r)
        a = math.acos(a)
        return (theta + a)

    def _solve_elbow_angle(self, l_one, l_two, r):
        a = (l_two ** 2 + l_one ** 2 - r ** 2) / (2 * l_one * l_two)
        a = math.acos(a)
        return a

    def inverse_solve(self, x_pos, y_pos, z_pos, roll, pitch):
        r, theta, phi = self._convert_to_spherical(x_pos, y_pos, z_pos)

        if r > self._max_range:
            pass # throw some form of exception indicating we can't reach the expected point

        angles = {}
        angles['s1'] = {'final_angle':phi}

        shoulder_angle = self._solve_shoulder_angle(theta, self._segment_info['a2']['segment_length'], self._segment_info['a3']['segment_length'], r)
        angles['s2'] = {'final_angle':shoulder_angle}

        elbow_angle = self._solve_elbow_angle(self._segment_info['a2']['segment_length'], self._segment_info['a3']['segment_length'], r)
        angles['s3'] = {'final_angle':elbow_angle}

        angles['s4'] = {'final_angle':roll}
        angles['s5'] = {'final_angle':pitch}

        return angles
