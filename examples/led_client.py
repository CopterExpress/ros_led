import rospy
from ros_ws281x.srv import SetLeds
from ros_ws281x.msg import LEDState, LEDStateArray
from std_msgs.msg import ColorRGBA

NUM_LEDS = 60

setLeds = rospy.ServiceProxy("/led/set_leds", SetLeds, persistent=True)


def fillStrip(red, green, blue):
    ledMsg = LEDStateArray()
    ledMsg.leds = []
    for i in range(NUM_LEDS):
        led = LEDState(i, ColorRGBA(red, green, blue, 0))
        ledMsg.leds.append(led)
    setLeds(ledMsg)


def fillColor(color, duration):
    currentIntensity = 0.0
    ledMessage = LEDStateArray()
    while currentIntensity < 1.0:
        currentIntensity += 1.0 / 60.0
        ledMessage.leds = []
        for i in range(NUM_LEDS):
            led = LEDState(i, ColorRGBA(color['r'] * currentIntensity,
                                        color['g'] * currentIntensity,
                                        color['b'] * currentIntensity,
                                        0))
            ledMessage.leds.append(led)
        setLeds(leds=ledMessage)
        rospy.sleep(duration / 60.0)


fillStrip(0, 0, 0)
fillColor({'r': 60.0, 'g': 0.0, 'b': 0.0}, 1.0)
rospy.sleep(1.0)
fillColor({'r': 0.0, 'g': 60.0, 'b': 0.0}, 1.0)
rospy.sleep(1.0)
fillColor({'r': 0.0, 'g': 0.0, 'b': 60.0}, 1.0)
rospy.sleep(1.0)
fillColor({'r': 0.0, 'g': 0.0, 'b': 0.0}, 0.2)

