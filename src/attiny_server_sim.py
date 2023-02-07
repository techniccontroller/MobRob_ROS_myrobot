#!/usr/bin/env python3
import time
import array
import threading

from myrobot_model.srv import AttinyCommand, AttinyCommandRequest, AttinyCommandResponse
import rospy

lock = threading.Lock()

upper_limit_VERT = 142
lower_limit_VERT = 29
upper_limit_GRIP = 153
lower_limit_GRIP = 0

current_pos_GRIP = 9999
current_pos_VERT = 9999
current_pos_SERVO = 9999

reponse_buffer = ""


def evaluate_input(input):

    global upper_limit_GRIP, upper_limit_VERT, \
            lower_limit_GRIP, lower_limit_VERT, \
            current_pos_GRIP, current_pos_VERT, current_pos_SERVO, \
            reponse_buffer

    parameter = int(input.rstrip()[6:-1])

    if input[0:2] == "sv":
        # Servo
        if input[3:5] == "wr":
            current_pos_SERVO = int(parameter)
        elif input[3:5] == "gp":
            reponse_buffer = "{:06d}".format(current_pos_SERVO)
    elif input[0:2] == "vt":
        # VERT
        if input[3:5] == "it":
            current_pos_VERT = upper_limit_VERT
        elif input[3:5] == "ma":
            if parameter > upper_limit_VERT:
                current_pos_VERT = upper_limit_VERT
            elif parameter < lower_limit_VERT:
                current_pos_VERT = lower_limit_VERT
            else:
                current_pos_VERT = parameter
        elif input[3:5] == "mr":
            current_pos_VERT +=parameter
            if current_pos_VERT > upper_limit_VERT:
                current_pos_VERT = upper_limit_VERT
            elif current_pos_VERT < lower_limit_VERT:
                current_pos_VERT = lower_limit_VERT
        elif input[3:5] == "gp":
            reponse_buffer = "{:06d}".format(current_pos_VERT)
    elif input[0:2] == "gr":
        # GRIP
        if input[3:5] == "it":
            current_pos_GRIP = 0
        elif input[3:5] == "ma":
            if parameter > upper_limit_GRIP:
                current_pos_GRIP = upper_limit_GRIP
            elif parameter < lower_limit_GRIP:
                current_pos_GRIP = lower_limit_GRIP
            else:
                current_pos_GRIP = parameter
        elif input[3:5] == "mr":
            current_pos_GRIP +=parameter
            if current_pos_GRIP > upper_limit_GRIP:
                current_pos_GRIP = upper_limit_GRIP
            elif current_pos_GRIP < lower_limit_GRIP:
                current_pos_GRIP = lower_limit_GRIP
        elif input[3:5] == "gp":
            reponse_buffer = "{:06d}".format(current_pos_GRIP)
        elif input[3:5] == "gg":
            is_grabbed = current_pos_GRIP == 0
            reponse_buffer = "{:06d}".format(is_grabbed)
    
    return input.upper()


def send_to_attiny(req):
    global spi, lock, reponse_buffer
    input = req.input
    with lock:
        print('\nRequest-Input', input.rstrip())
        str = evaluate_input(input)
        time.sleep(0.02)
        print("attiny: ", str[:-1])
        if "gp" in input or "gg" in input:
            time.sleep(0.02)
            str = reponse_buffer + "\n"
            print("attiny-res: ", str[:-1])
    return AttinyCommandResponse(str[:-1])
    

if __name__ == "__main__":

    rospy.init_node('attiny_server_sim')
    s = rospy.Service('attiny_command', AttinyCommand, send_to_attiny)
    print("Ready to send commands.")
    rospy.spin()