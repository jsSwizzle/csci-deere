import math
import numpy as np

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

        theta = z_pos / r
        theta = math.acos(theta)

        phi = y_pos / z_pos
        phi = math.atan(phi)

        return r, theta, phi

    def _solve_shoulder_angle(self, theta, l_one, l_two, r):
        a = (l_one ** 2 + r ** 2 - l_two ** 2) / (2 * l_one * r)
        a = math.acos((a % 2) - 1)
        return (theta + a)

    def _solve_elbow_angle(self, l_one, l_two, r):
        a = (l_two ** 2 + l_one ** 2 - r ** 2) / (2 * l_one * l_two)
        a = math.acos((a % 2) - 1)
        return a

    def specific_inverse_solve(self, x_pos, y_pos, z_pos, roll, pitch):
        """Finds the angles for each joint of the arm given a target end effector.

        Calculates and the angles each joint needs to be at given the target end
        effector (x_pos, y_pos, z_pos) in cartesion space. Placing the wrist in the
        position so it has the given roll and pitch.

        Args:
            x_pos {float} -- target x position for the end effector in cartesian space.
            y_pos {float} -- target y position for the end effector in cartesian space.
            z_pos {float} -- target z position for the end effector in cartesian space.
            roll {float} -- end roll angle for the wrist when it is at the end effector.
            pitch {float} -- end pitch angle for the wrist when it is at the end effector.

        Returns:
            angles {dict} -- list of angles for each servo in the arm (angles[<servoID>]['final_angle'])
        """
        r, theta, phi = self._convert_to_spherical(x_pos, y_pos, z_pos)

        if r > self._max_range:
            pass # throw some form of exception indicating we can't reach the expected point

        angles = {}
        angles['s1'] = {'final_angle':phi}

        shoulder_angle = self._solve_shoulder_angle(theta, self._segment_info['seg2']['segment_length'], self._segment_info['seg3']['segment_length'], r)
        angles['s2'] = {'final_angle':shoulder_angle}

        elbow_angle = self._solve_elbow_angle(self._segment_info['seg2']['segment_length'], self._segment_info['seg3']['segment_length'], r)
        angles['s3'] = {'final_angle':elbow_angle}

        angles['s4'] = {'final_angle':roll}
        angles['s5'] = {'final_angle':pitch}

        return angles

    def forward_solve(self, current_angles):
        """Finds the (x, y, z) position of the end effector of the arm.

        Calculates the current (x, y, z) position of the end effector of the arm
        using the given angles of each of the joints.

        Args:
            current_angles {dict} -- dictionary list of current angles for each servo (key = servoID, value = current angle)

        Returns:
            (x_pos, y_pos, z_pos) {list} -- list of individual elements for each coordinate in the cartesian plane
        """
        pass
