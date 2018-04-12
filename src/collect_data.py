#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
import pandas as pd
import message_filters
import time 

steer_val = 90
bridge = CvBridge()
image_paths = []
steer_values = []

def callback_img(image):
    img = bridge.imgmsg_to_cv2(image, "bgr8")
    file_path = str(time.time()) + '.jpg'
    img = cv2.resize(img, (200,66), cv2.INTER_AREA)
    cv2.imwrite(('../images/'+file_path), img)
    image_paths.append(file_path)
    steer_values.append(steer_val)

def callback_steer(steer):
    global steer_val
    steer_val = steer.data

def shutdown():
    dataframe = pd.DataFrame({'image_paths': image_paths, 'steer_values': steer_values})
    dataframe.to_csv('../images/image_and_steer' + str(time.time())+'.csv')
    print('Saved Data!')

def main():
    
    rospy.init_node('data_collect_node')
    image_sub = rospy.Subscriber('center_cam', Image, callback_img, queue_size=1, buff_size = 2**24)
    steer_sub = rospy.Subscriber('steer_return', Int16, callback_steer, queue_size = 1)
    rospy.spin()
    rospy.on_shutdown(shutdown)

if __name__ == "__main__":
    main()
    
