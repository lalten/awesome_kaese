#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_analog_out_v2 import BrickletAnalogOutV2
from tinkerforge.brick_silent_stepper import BrickSilentStepper
from tinkerforge.bricklet_analog_in_v3 import BrickletAnalogInV3

import numpy as np

import rospy

from geometry_msgs.msg import Twist


class Hwif:
    def __init__(self):
        self.input_topic = rospy.get_param('~input_topic', '/cmd_vel')
        self.analog_out_UID = rospy.get_param('~tinkerforge_analog_out_UID', 'Boq')
        self.stepper_left_UID = rospy.get_param('~tinkerforge_stepper_left_UID', '6QEPuu')
        self.stepper_right_UID = rospy.get_param('~tinkerforge_stepper_right_UID', '6rnvVj')
        self.analog_in_UID = rospy.get_param('~tinkerforge_analog_in_UID', 'F52')
        self.tinkerforge_host = rospy.get_param('~tinkerforge_host', 'localhost')
        self.tinkerforge_port = rospy.get_param('~tinkerforge_port', 4223)

        self.actual_steering = 0

        self.sub = rospy.Subscriber(self.input_topic, Twist, self.cmd_vel_callback, queue_size=10)

        self.ipcon = IPConnection()  # Create IP connection
        self.ao = BrickletAnalogOutV2(self.analog_out_UID, self.ipcon)
        self.ai = BrickletAnalogInV3(self.analog_in_UID, self.ipcon)
        self.ipcon.connect(self.tinkerforge_host, self.tinkerforge_port)

        self.stepper_left = BrickSilentStepper(self.stepper_left_UID, self.ipcon)
        self.stepper_right = BrickSilentStepper(self.stepper_right_UID, self.ipcon)

        self.steering_wheel_angle = 0

        self.stepper_motor_current = 800
        self.stepper_max_velocity = 2000
        self.stepper_speed_ramping_accel = 0
        self.stepper_speed_ramping_deaccel = 0

        self.stepper_left.set_motor_current(self.stepper_motor_current)  # 800mA
        self.stepper_left.set_step_configuration(self.stepper_left.STEP_RESOLUTION_8, True) # 1/8 steps (interpolated)
        self.stepper_left.set_max_velocity(self.stepper_max_velocity)
        self.stepper_left.set_speed_ramping(self.stepper_speed_ramping_accel, self.stepper_speed_ramping_deaccel)
        self.stepper_left.enable()

        self.stepper_right.set_motor_current(self.stepper_motor_current)  # 800mA
        self.stepper_right.set_step_configuration(self.stepper_right.STEP_RESOLUTION_8, True) # 1/8 steps (interpolated)
        self.stepper_right.set_max_velocity(self.stepper_max_velocity)
        self.stepper_right.set_speed_ramping(self.stepper_speed_ramping_accel, self.stepper_speed_ramping_deaccel)
        self.stepper_right.enable()

        self.ai.register_callback(self.ai.CALLBACK_VOLTAGE, self.steering_callback)
        self.ai.set_voltage_callback_configuration(50, False, "x", 0, 0)

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
        # rospy.loginfo('sent {}'.format(speed))

    def steering_callback(self, millivolt):
        lin = np.log(millivolt)
        self.actual_steering = self.valmap(lin, np.log(70), np.log(250), -1.0, 1.0)
        # rospy.loginfo('got {}mV --> lin {}, steer {}'.format(millivolt, lin, self.actual_steering))


if __name__ == '__main__':
    rospy.init_node('kaercher_hwif')
    n = Hwif()
    rospy.spin()
