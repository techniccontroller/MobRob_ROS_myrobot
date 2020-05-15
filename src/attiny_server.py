#!/usr/bin/env python
import time
import spidev
import array
from subprocess import call, PIPE

from myrobot_model.srv import AttinyCommand, AttinyCommandRequest, AttinyCommandResponse
import rospy


def send_to_attiny(req):
    global spi
    input = req.input
    print 'Request-Input', input
    if input[-1] != '\n':
        input += '\n'
    chars = []
    for c in input:
            chars.append(ord(c))
    res = spi.xfer2(chars)
    time.sleep(0.1)
    str = ''
    for i in res:
        str += chr(i)
    print "attiny: ", str[1:]
    if "gp" in input:
        time.sleep(0.1)
        chars = [48,48,48,48,48,48,48]
        res = spi.xfer2(chars)
        str = ''
        for i in res:
            str += chr(i)
        print "attiny-res: ", str[1:]
    return AttinyCommandResponse(str[1:])
    

if __name__ == "__main__":

    global spi

    # Reset Attiny
    call("gpio -g mode 22 out", shell = True)
    call("gpio -g write 22 0", shell = True)
    time.sleep(0.1)
    call("gpio -g write 22 1", shell = True)

    # Open SPI interface
    spi = spidev.SpiDev()
    spi.open(0,0)
    spi.max_speed_hz = 10000
    spi.mode = 0b00

    rospy.init_node('attiny_server')
    s = rospy.Service('attiny_command', AttinyCommand, send_to_attiny)
    print "Ready to send commands."
    rospy.spin()