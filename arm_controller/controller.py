from adafruit_servokit import ServoKit

class Controller:

    def __init__(self):
        """Basic constructor for Controller class.
        """
        servo_info = {}
        servo_info['ss'] = {'function':'speed','default_value':0.0}
        servo_info['s1'] = {'function':'waist','default_value':90.0}
        servo_info['s6'] = {'function':'grip','default_value':80.0}
        self._servo_info = servo_info
        self._kit = ServoKit(channels=16)

    def configure_board(self, mapping = {0:'s1',1:'s2',2:'s3',3:'s4',4:'s5',5:'s6'}):
        """Sets up servo mapping.

        Keyword Arguments:
            mapping {dict} --
        """
        for servo_no,servo_id in mapping.items():
            self._servo_info[servo_id]['servo#'] = servo_no

    def set_default_position(self):
        """Loads the default position for the robot arm.
        """
        for k,v in self._servo_info.items():
            self.set_robot(k,v['default_value'])

    def set_robot(self, part, value):
        """Moves the robot.

        Moves the specified part to the given value.

        Arguments:
            part {str} -- item to move
            value {float} -- value to apply to part
        """
        if part != 'ss':
            self._kit.servo[self._servo_info[part]['servo#']].angle = value
