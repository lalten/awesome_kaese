#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_analog_out_v2 import BrickletAnalogOutV2

import rospy

from geometry_msgs.msg import Twist


class Hwif:
    def __init__(self):
        self.input_topic = rospy.get_param('~input_topic', '/cmd_vel')
        self.analog_out_UID = rospy.get_param('~tinkerforge_analog_out_UID', 'Boq')
        self.tinkerforge_host = rospy.get_param('~tinkerforge_host', 'localhost')
        self.tinkerforge_port = rospy.get_param('~tinkerforge_port', 4223)

        self.sub = rospy.Subscriber(self.input_topic, Twist, self.cmd_vel_callback, queue_size=10)

        self.ipcon = IPConnection()  # Create IP connection
        self.ao = BrickletAnalogOutV2(self.analog_out_UID, self.ipcon)
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

    def cmd_vel_callback(self, cmd_vel_msg):
        speed = cmd_vel_msg.linear.x
        if speed < 0:
            speed = 0
        if speed > 1:
            speed = 1
        speed = self.valmap(speed, 0, 1, 0, 3300)  # convert to mV Potentiometer emulation
        self.ao.set_output_voltage(speed)
        rospy.loginfo('sent {}'.format(speed))


if __name__ == '__main__':
    rospy.init_node('kaercher_hwif')
    n = Hwif()
    rospy.spin()
