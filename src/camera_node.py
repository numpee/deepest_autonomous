#!/usr/bin/env python
# ROS code for sending image from RASPI Cam


import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

# initialize camers

rospy.loginfo('Init camera')
cam = cv2.VideoCapture(0)

def main():
    image_pub = rospy.Publisher('center_cam', Image, queue_size = 1)
    rospy.init_node('camera_node', anonymous = True)
    rate = rospy.Rate(10)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        _, frame = cam.read()
        front_img = bridge.cv2_to_imgmsg(frame, "bgr8")
        rospy.loginfo("images sent")
        image_pub.publish(front_img)
        rate.sleep()
    
    cam.release()
    print('terminate loop')

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("terminated")