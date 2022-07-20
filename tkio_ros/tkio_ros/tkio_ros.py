from math import degrees
import rclpy
import rclpy.logging
import rclpy.qos
import rclpy.action
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time, Duration

from ackermann_msgs.msg import AckermannDrive

import serial

class TkioRos(Node):

    def __init__(self):
        super().__init__('TkioRos')

        # 30mph, this is in mps
        self.MAX_SPEED = 13.4112 #TODO double check this

        self.nav_sub = self.create_subscription(
            AckermannDrive, '/nav_vel', self.nav_callback, rclpy.qos.qos_profile_system_default)

        self.serial = serial.Serial('ttyUSB0', baudrate=9600)

    def nav_callback(self, msg: AckermannDrive):
        """ Send messages over uart to pico whenever we have a new ackermann nav command. """

        # Cap speed
        if msg.speed > self.MAX_SPEED:
            msg.speed = self.MAX_SPEED

        # Get speed as a percentage for tkio
        speed_str = str(msg.speed / self.MAX_SPEED * 100)
        # tkio uses the same right left convention as Ros, so no conversion needed
        steering_str = str(degrees(msg.steering_angle))

        # we need to send two commands
        speed_to_write = b''
        angle_to_write = b''

        if msg.speed > 0:
            speed_to_write = ('F ' + speed_str + '!').encode('ascii')
        elif msg.speed < 0:
            speed_to_write = ('R ' + speed_str + '!').encode('ascii')
        elif msg.speed == 0:
            speed_to_write = 'N!'.encode('ascii') 

        angle_to_write = ('S ' + steering_str + '!').encode('ascii')

        # write angle first, as the two messages will be slightly delayed
        self.serial.write(angle_to_write)
        self.serial.write(speed_to_write)

