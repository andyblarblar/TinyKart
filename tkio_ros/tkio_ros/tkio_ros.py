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
        
        self.last_was_rev = False
        self.last_rev_speed = 0.0

        self.nav_sub = self.create_subscription(
            AckermannDrive, '/ack_vel', self.nav_callback, rclpy.qos.qos_profile_system_default)

        self.serial = serial.Serial('/dev/ttyUSB0', baudrate=9600)

    def nav_callback(self, msg: AckermannDrive):
        """ Send messages over uart to pico whenever we have a new ackermann nav command. """

        # Cap speed
        if msg.speed > self.MAX_SPEED:
            msg.speed = self.MAX_SPEED

        # Get speed as a percentage for tkio
        speed_str = str(int(msg.speed / self.MAX_SPEED * 100))
        # tkio uses the same right left convention as Ros, so no conversion needed
        steering_str = str(int(degrees(msg.steering_angle)))

        # we need to send two commands
        speed_to_write = b''
        angle_to_write = b''

        if msg.speed > 0:
            speed_to_write = ('F ' + speed_str + '!').encode('ascii')
            self.last_was_rev = False
        elif msg.speed < 0 and self.last_was_rev and self.last_rev_speed == msg.speed:
            # NOP to avoid spamming reverse if staying in reverse at the same speed, this prevents stuttering.
            angle_to_write = ('S ' + steering_str + '!').encode('ascii')
            self.serial.write(angle_to_write)
            return
        elif msg.speed < 0:
            # split to remove -, which is implied in reverse
            speed_to_write = ('R ' + speed_str.rsplit('-')[1] + '!').encode('ascii')
            self.last_rev_speed = msg.speed
            self.last_was_rev = True
        elif msg.speed == 0:
            speed_to_write = 'N!'.encode('ascii') 
            self.last_was_rev = False

        angle_to_write = ('S ' + steering_str + '!').encode('ascii')

        # Debug
        self.get_logger().info(f"{angle_to_write} | {speed_to_write}")

        # write angle first, as the two messages will be slightly delayed
        self.serial.write(angle_to_write)
        self.serial.write(speed_to_write)

