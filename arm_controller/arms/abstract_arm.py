"""Abstract Base Class for all robot arms.
"""
from abc import ABC,abstractmethod

class AbstractArm(ABC):

    @abstractmethod
    def __init__(self):
        """Constructs Robot class.
        """

    @abstractmethod
    def moveto(self, x_pos, y_pos, z_pos):
        """Moves the arm to the specified position.

        Calculates and moves the arm so the claw is centered at the
        position (x_pos, y_pos, z_pos).

        Args:
            x_pos {float} -- Final X position of the claw.
            y_pos {float} -- Final Y position of the claw.
            z_pos {float} -- Final Z position of the claw.
        """
        pass

    @abstractmethod
    def getpos(self):
        """Calculates and returns current position of the arm.

        Calculates based on current positions of arm servo's, the
        current position of the claw, returning as (x, y, z).

        Return:
            A list containing the (x, y, z) position of the claw.
        """
        pass

    @abstractmethod
    def setspeed(self, s):
        """Set's the speed at which the servo's move.

        Set's the arm rate of speed at which the servo's move into position.

        Args:
            s {int} -- Rate of speed on a 1-10 scale: 1 being slowest, 10 being fastest.
        """
        pass
