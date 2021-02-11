from kinem import *
from controller import *

class Arm:

    def __init__(self):
        """Constructs Arm class.
        """

    def moveto(self, x_pos, y_pos, z_pos):
        """Moves the arm to the specified position.

        Calculates and moves the arm so the claw is centered at the
        position (x_pos, y_pos, z_pos).

        Args:
            x_pos {float} -- Final X position of the claw.
            y_pos {float} -- Final Y position of the claw.
            z_pos {float} -- Final Z position of the claw.
        """
        return 0

    def getpos(self):
        """Calculates and returns current position of the arm.

        Calculates based on current positions of arm servo's, the
        current position of the claw, returning as (x, y, z).

        Return:
            A list containing the (x, y, z) position of the claw.
        """
        return 0

    def setspeed(self, s):
        """Set's the speed at which the servo's move.
        """
        return 0

    def closeclaw(self, p=100):
        return 0

    def openclaw(self):
        return 0
