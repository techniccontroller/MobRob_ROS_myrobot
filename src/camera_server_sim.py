#!/usr/bin/env python3
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
import argparse
import socket
import cv2
import base64
import netifaces as ni
import numpy as np

frame = np.zeros((640, 480, 3))

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    global frame
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2_img
    except CvBridgeError as e:
        print(e)
    else:
        frame = cv2_img

def read_frame_from_camera(scale, rotate, encode_param):
    global frame
    # resizing of frame
    frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    if rotate:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
    # encode image as base64 String
    result, img_encoded = cv2.imencode('.jpg', frame, encode_param)
    stringData = base64.b64encode(img_encoded)
    return stringData


def start_server(ip, port, scale, rotate):
    # encode parameters for image conversion
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

    print("open TCP server socket")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((ip, port))
    print("Waiting for TCP client ...")
    sock.listen(True)
    conn, addr = sock.accept()
    print("Connected: " + addr[0])

    stringData = read_frame_from_camera(scale, rotate, encode_param)

    while True:
        try:
            input_command = conn.recv(11)
        except socket.error:
            input_command = "error"
            print("Lost connection...")
        if input_command == b'getNewFrame':
            stringData = read_frame_from_camera(scale, rotate, encode_param)

            # send captured frame to server
            conn.send(bytes(str(len(stringData)).ljust(16), encoding='utf-8'))
            conn.send(stringData)

        elif input_command == b'getCacheFra':
            # send previous captured frame to server
            conn.send(bytes(str(len(stringData)).ljust(16), encoding='utf-8'))
            conn.send(stringData)

            stringData = read_frame_from_camera(scale, rotate, encode_param)

        elif input_command == b'closeDriver':
            break
        else:
            print("Waiting for TCP client ...")
            sock.listen(True)
            conn, addr = sock.accept()
            print("Connected: " + addr[0])

    sock.close()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # construct the argument parser and parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--interface", type=str, default='wlan0',
                    help="interface where the server should be hosted, default: wlan0")
    ap.add_argument("-p", "--port", type=int, default=5001,
                    help="ephemeral port number of the server (1024 to 65535), default: 5001")
    ap.add_argument("-s", "--scale", type=float, default=1.0,
                    help="which scaling factor should be applied to the image, default: 1.0")
    ap.add_argument("-r", "--rotate", type=bool, default=True,
                    help="whether the image should be rotated by 180 degree, default: False")
    args = vars(ap.parse_args())

    print("Start camera TCP server with follow settings: ")
    print('Interface: ', args['interface'])
    print('Port: ', args['port'])
    print('Scale: ', args['scale'])
    print('Rotate: ', args['rotate'])

    try:
        ip_address = ni.ifaddresses(args['interface'])[ni.AF_INET][0]['addr']
    except Exception as e:
        print(e)
        ip_address = "127.0.0.1"

    print(f"IP Address: {ip_address}")

    rospy.init_node('camera_server_sim')
    # Define your image topic
    image_topic = "/mobrob/camera1/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)

    start_server(ip_address, args['port'], args['scale'], args['rotate'])