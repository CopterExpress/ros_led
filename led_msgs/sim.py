#!/usr/bin/env python

# Simulator of a LED strip.
# Note this should be run in a terminal supporting 24 bit color.

import sys
import rospy
from led_msgs.srv import SetLed, SetLeds


rospy.init_node('led')
led_count = rospy.get_param('~led_count', 30)
state = [{'r': 0, 'g': 0, 'b': 0}] * led_count


def set_led(req):
    state[req.index] = {'r': int(req.r), 'g': int(req.g), 'b': int(req.b)}
    print_led()
    return {'success': True}


def set_leds(req):
    for led in req.leds:
        state[led.index] = {'r': int(led.r), 'g': int(led.g), 'b': int(led.b)}
    print_led()
    return {'success': True}


rospy.Service('~set_led', SetLed, set_led)
rospy.Service('~set_leds', SetLeds, set_leds)


def print_led():
    s = ''
    for led in state:
        s += '\033[48;2;{};{};{}m '.format(led['r'], led['g'], led['b'])
    sys.stdout.write('\r{}\033[0m'.format(s))
    sys.stdout.flush()


print_led()
rospy.spin()
