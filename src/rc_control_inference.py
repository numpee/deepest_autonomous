#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import Int16


cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 300)

def main():
    rospy.init_node("inference_node")
    steer_pub = rospy.Publisher("/cmd_vel", Int16, queue_size=5)

    while not rospy.is_shutdown():
        _, img = cam.read()
        img = cv2.resize(img, (200,66), cv2.INTER_AREA)

        '''
        TENSORFLOW INFERENCE CODE HERE
        steer = predict(img)
        comment steer=0.
        '''
        steer=0
        steer_pub.publish(steer)

    cam.release()

if __name__ == '__main__':
    main()

