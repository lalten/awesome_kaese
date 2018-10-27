#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_joystick import BrickletJoystick

import numpy as np

import rospy

from geometry_msgs.msg import Twist


class Joy:
    def __init__(self):
        self.output_topic = rospy.get_param('~output_topic', '/cmd_vel')
        self.joy_UID = rospy.get_param('~tinkerforge_joy_UID', 'waz')
        self.tinkerforge_host = rospy.get_param('~tinkerforge_host', 'localhost')
        self.tinkerforge_port = rospy.get_param('~tinkerforge_port', 4223)

        self.actual_steering = 0

        self.pub = rospy.Publisher(self.output_topic, Twist, queue_size=1)

        self.ipcon = IPConnection()  # Create IP connection
        self.joy = BrickletJoystick(self.joy_UID, self.ipcon)

        self.ipcon.connect(self.tinkerforge_host, self.tinkerforge_port)

    @staticmethod
    def valmap(ivalue, istart, istop, ostart, ostop):
        """
        map a value from one input range to output range, like the Arduino map function
        :param ivalue: input value to map
        :param istart: input from
        :param istop: input to
        :param ostart: output from
        :param ostop:  output to
        :return: mapped input
        """
        return ostart + (ostop - ostart) * ((ivalue - istart) / (istop - istart))

    def run(self):
        cmd_vel = Twist()
        while not rospy.is_shutdown():
            x, y = self.joy.get_position()
            pressed = self.joy.is_pressed()

            # if not pressed:
            #     cmd_vel.linear.x = 0
            # else:
            cmd_vel.linear.x = self.valmap(y, 100, -100, -1, 1)
            cmd_vel.angular.z = self.valmap(x, -100, 100, -1, 1)

            self.pub.publish(cmd_vel)


if __name__ == '__main__':
    rospy.init_node('kaercher_joy')
    n = Joy()
    n.run()
