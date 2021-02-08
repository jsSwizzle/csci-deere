"""App to be ran on Raspberry pi for controlling robot via bluetooth app
"""
import os
import glob
import time
from bluetooth import *
from adafruit_servokit import ServoKit


class Robot(object):
    def __init__(self):
        """Basic constructor

        Arguments:
            object {Robot} -- robot arm
        """
        servo_info = {}
        servo_info['ss'] = {'function':'speed','default_value':0.0}
        servo_info['s1'] = {'function':'waist','default_value':90.0}
        servo_info['s2'] = {'function':'shoulder','default_value':150.0}
        servo_info['s3'] = {'function':'elbow','default_value':35.0}
        servo_info['s4'] = {'function':'wrist_roll','default_value':140.0}
        servo_info['s5'] = {'function':'wrist_pitch','default_value':85.0}
        servo_info['s6'] = {'function':'grip','default_value':80.0}
        self._servo_info = servo_info
        self._kit = ServoKit(channels=16)
        self.configure_board()


    def set_default_position(self):
        """Loads default positions for robot
        """
        for k,v in self._servo_info.items():
            self.set_robot(k,v['default_value'])


    def configure_board(self,mapping = {0:'s1',1:'s2',2:'s3',3:'s4',4:'s5',5:'s6'}):
        """Sets up the mapping from received channels from android to raspberry pi servos

        Keyword Arguments:
            mapping {dict} -- mapping from board to android channels (default: {{0:'s1',1:'s2',2:'s3',3:'s4',4:'s5',5:'s6'}})
        """
        for servo_no,servo_id in mapping.items():
            self.servo_info[servo_id]['servo#'] = servo_no


    def set_robot(self,part,value):
        """Moves the robot

        Arguments:
            part {str} -- item to move
            value {float} -- value to apply to part
        """
        if part != 'ss':
            self._kit.servo[self._servo_info[part]['servo#']].angle = value


    def on_android_app_rcv(self,msg):
        # first two characters are command, remaining is value
        command = msg[:2]
        value   = float(msg[2:])
        assert(command in self._servo_info), f"Invalid command '{command}' received!"
        self.set_robot(command,value)


if __name__=="__main__":
    robo = Robot()
    connection = False
    server_sock = BluetoothSocket( RFCOMM )
    server_sock.bind(("",PORT_ANY))
    server_sock.listen(1)

    port = server_sock.getsockname()[1]

    uuid = "00001101-0000-1000-8000-00805f9b34fb"

    advertise_service( server_sock, "VoltMeterPiServer",
                    service_id = uuid,
                    service_classes = [ uuid, SERIAL_PORT_CLASS ],
                    profiles = [ SERIAL_PORT_PROFILE ] 
                        )
    while True:
        if(connection == False):
            print("Waiting for connection on RFCOMM channel %d" % port)
            client_sock, client_info = server_sock.accept()
            connection = True
            print("Accepted connection from ", client_info)
        try:
            data = client_sock.recv(1024)
            functionality = data[:1]
            robo.on_android_app_rcv(data)
        except:
            print('error')