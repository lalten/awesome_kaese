#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_analog_out_v2 import BrickletAnalogOutV2
from tinkerforge.brick_silent_stepper import BrickSilentStepper
from tinkerforge.bricklet_analog_in_v3 import BrickletAnalogInV3
from tinkerforge.bricklet_industrial_dual_relay import BrickletIndustrialDualRelay

import numpy as np

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class Hwif:
    def __init__(self):
        self.input_topic = rospy.get_param('~input_topic', '/cmd_vel')
        self.override_topic = rospy.get_param('~override_topic', '/override')
        self.analog_out_UID = rospy.get_param('~tinkerforge_analog_out_UID', 'Boq')
        self.relay_UID = rospy.get_param('~tinkerforge_relay_UID', 'FMY')
        self.stepper_left_UID = rospy.get_param('~tinkerforge_stepper_left_UID', '6QEPuu')
        self.stepper_right_UID = rospy.get_param('~tinkerforge_stepper_right_UID', '6rnvVj')
        self.analog_in_UID = rospy.get_param('~tinkerforge_analog_in_UID', 'F52')
        self.tinkerforge_host = rospy.get_param('~tinkerforge_host', 'localhost')
        self.tinkerforge_port = rospy.get_param('~tinkerforge_port', 4223)

        self.max_speed_millivolt = rospy.get_param('~max_speed_millivolt', 300)
        self.controller_p = rospy.get_param('~controller_p', 500.0)
        self.deadzone = rospy.get_param('~deadzone', 0.10)

        self.actual_steering = 0

        self.sub = rospy.Subscriber(self.input_topic, Twist, self.cmd_vel_callback, queue_size=10)
        self.sub_override = rospy.Subscriber(self.override_topic, Bool, self.override_callback, queue_size=10)
        self.override = False

        self.ipcon = IPConnection()  # Create IP connection
        self.ao = BrickletAnalogOutV2(self.analog_out_UID, self.ipcon)
        self.idr = BrickletIndustrialDualRelay(self.relay_UID, self.ipcon)
        self.ai = BrickletAnalogInV3(self.analog_in_UID, self.ipcon)
        self.stepper_left = BrickSilentStepper(self.stepper_left_UID, self.ipcon)
        self.stepper_right = BrickSilentStepper(self.stepper_right_UID, self.ipcon)
        self.ai.register_callback(self.ai.CALLBACK_VOLTAGE, self.steering_callback)

        self.ipcon.connect(self.tinkerforge_host, self.tinkerforge_port)

        self.ai.set_voltage_callback_configuration(50, False, "x", 0, 0)

        self.ao.set_output_voltage(0)
        self.idr.set_value(False, False)

        self.actual_steering = 0
        self.steering_setpoint = 0

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

        self.controller_timer = rospy.Timer(period=rospy.Duration(1/200.0), callback=self.controller_timer_callback)

    def override_callback(self, msg):
        self.override = msg.data
        self.ao.set_output_voltage(0)

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
        # stop on override
        if self.override:
            self.ao.set_output_voltage(0)
            return

        # extract steering angle and forward speed from msg
        self.steering_setpoint = cmd_vel_msg.angular.z
        speed = cmd_vel_msg.linear.x

        # If speed is not zero, activate gas pedal switch
        if np.abs(speed) > self.deadzone:
            self.idr.set_selected_value(0, True)
        else:
            self.idr.set_selected_value(0, False)

        # If speed is backwards, switch relay to backward drive
        if speed < 0:
            self.idr.set_selected_value(1, False)
        else:
            self.idr.set_selected_value(1, True)
        speed = np.abs(speed)

        # Clamp to max speed
        if speed > 1:
            speed = 1

        # Convert to millivolt for DAC
        millivolts = self.valmap(speed, 0, 1, 0, self.max_speed_millivolt)  # convert to mV Potentiometer emulation
        if millivolts < 0:
            millivolts = 0
        if millivolts > self.max_speed_millivolt:
            millivolts = self.max_speed_millivolt

        # send to output
        self.ao.set_output_voltage(millivolts)

        # rospy.loginfo('sent {}'.format(millivolts))

    def steering_callback(self, millivolt):
        lin = np.log(millivolt)
        self.actual_steering = self.valmap(lin, np.log(70), np.log(200), -1.0, 1.0)
        rospy.loginfo('got {}mV --> lin {}, steer {}'.format(millivolt, lin, self.actual_steering))

    def controller_timer_callback(self, e):
        self.controller_p = rospy.get_param('~controller_p', 50.0)
        err = self.actual_steering - self.steering_setpoint
        steps = self.controller_p * err
        if self.override:
            steps = 0
        self.stepper_left.set_steps(steps)
        self.stepper_right.set_steps(steps)
        # rospy.loginfo('actual: {}, setpoint: {}'.format(self.actual_steering, self.steering_setpoint))


if __name__ == '__main__':
    rospy.init_node('kaercher_hwif')
    n = Hwif()
    rospy.spin()
