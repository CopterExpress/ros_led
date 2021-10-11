import rospy
from led_msgs.srv import SetLEDs
from led_msgs.msg import LEDStateArray, LEDState

rospy.init_node('flight')

set_leds = rospy.ServiceProxy('led/set_leds', SetLEDs)  # define proxy to ROS service

# switch LEDs number 0, 1 and 2 to red, green and blue color:


set_leds([LEDState(1, 255, 0, 0), LEDState(41, 0, 255, 0), LEDState(42, 0, 0, 255)])
