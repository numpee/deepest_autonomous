import tensorflow as tf
import numpy as np
from models import *

#teacher_depthwise = Teacher_depthwise()
teacher = Teacher()
student = Student(teacher, width=0.5)

with tf.Session() as sess:
    sess.run(tf.global_variables_initializer())

    saver = tf.train.Saver()
    save_path = saver.save(sess, "../Model_checkpoints/student")
    print("model saved in path: %s" % save_path)
