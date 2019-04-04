import rospy
from ros_ws281x import srv, msg

NUM_LEDS = 11

setLeds = rospy.ServiceProxy("/led/set_leds", srv.SetLeds, persistent=True)


def fillColor(color, duration):
    currentIntensity = 0.0
    Led = msg.LEDState
    ledMessage = msg.LEDStateArray()
    while currentIntensity < 1.0:
        currentIntensity += 1.0 / 60.0
        ledMessage.leds = []
        for i in range(NUM_LEDS):
            led = Led()
            led.index = i
            led.color.r = color['r'] * currentIntensity
            led.color.g = color['g'] * currentIntensity
            led.color.b = color['b'] * currentIntensity
            ledMessage.leds.append(led)
        setLeds(leds = ledMessage)
        rospy.sleep(duration / 60.0)


fillColor({'r': 60.0, 'g': 0.0, 'b': 0.0}, 1.0)
rospy.sleep(1.0)
fillColor({'r': 0.0, 'g': 60.0, 'b': 0.0}, 1.0)
rospy.sleep(1.0)
fillColor({'r': 0.0, 'g': 0.0, 'b': 60.0}, 1.0)
rospy.sleep(1.0)
fillColor({'r': 0.0, 'g': 0.0, 'b': 0.0}, 0.2)
