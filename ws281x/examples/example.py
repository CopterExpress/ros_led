import rospy
from led_msgs.srv import SetLEDs
from led_msgs.msg import LEDStateArray, LEDState
import time

led_count = rospy.get_param('led/led_count')
set_leds = rospy.ServiceProxy('led/set_leds', SetLEDs, persistent=True)

def fill_strip(red, green, blue):
    print(set_leds(leds=[LEDState(i, red, green, blue) for i in range(led_count)]))
    
def colorWipe(red, green, blue, wait_ms=30):
    for i in range(led_count):
        set_leds(leds=[LEDState(i, red, green, blue)])
        time.sleep(wait_ms / 1000.0)
        
def wheel(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos < 85:
        return Color(pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return Color(255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return Color(0, pos * 3, 255 - pos * 3)
        
while True:
    #colorWipe(255,0,0)
    #colorWipe(0,255,0)
    colorWipe(0,0,255)
    #fill_strip(200, 200, 200)
    #rospy.sleep(1)
    #fill_strip(0, 200, 0)
    #rospy.sleep(1)

