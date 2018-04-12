#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import Int16
import time


cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 300)

'''
def main():
    rospy.init_node("inference_node")
    steer_pub = rospy.Publisher("/cmd_vel", Int16, queue_size=5)

    while not rospy.is_shutdown():
        _, img = cam.read()
        img = cv2.resize(img, (200,66), cv2.INTER_AREA)

        #TENSORFLOW INFERENCE CODE HERE
        #steer = predict(img)
        #comment steer=0.
        
        steer=0
        steer_pub.publish(steer)

    cam.release()
'''
def main():
    rospy.init_node("inference_node")
    steer_pub = rospy.Publisher("/cmd_vel", Int16, queue_size=5)

    # declare models
    tf.reset_default_graph()    

    teacher = Teacher() #or Teacher_depthwise
    student = Student(teacher, width = 0.5)   
    with tf.Session() as sess:
        tf.global_variables_initializer().run()
        saver = tf.train.Saver()
        saver.restore(sess, "../Model_checkpoints/student") #change here to teacher, teacher_depthwise, student, student_pruned
    
        while not rospy.is_shutdown():
            _, img = cam.read()
            img = cv2.resize(img, (66,200), cv2.INTER_AREA)

            #TENSORFLOW INFERENCE CODE HERE
            steer, inference_time = predict(sess, model, img)
            #comment steer=0.

            #steer=0
            steer_pub.publish(steer)

    cam.release()

def predict(sess, model, image):
    feed_dict = {model.input : [image]}
    start_time = time.time()
    steering = sess.run([model.steering], feed_dict = feed_dict)
    steering = steering[0][0][0]
    inference_time = time.time() - start_time
    #print("steering: ", steering, ", inference_time: ", time.time()-start_time)
    return steering, inference_time
    
if __name__ == '__main__':
    main()

