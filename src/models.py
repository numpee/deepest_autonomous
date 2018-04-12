#!/usr/bin/env python
import tensorflow as tf
import numpy as np

initializer = tf.contrib.layers.xavier_initializer()

class Teacher(object):
    def __init__(self):
        self.input = tf.placeholder(tf.float32, shape=(None, 66, 200, 3), name='teacher_input')
        self.steering_ref = tf.placeholder(tf.float32, shape=(None, 1), name='teacher_steering')

        # Convolutional layers
        W_conv0 = tf.get_variable('Teacher/W_conv0', shape=[5, 5, 3, 24],
                                  initializer=initializer)
        b_conv0 = tf.get_variable('Teacher/b_conv0', shape=[24], initializer=initializer)
        tmp = tf.nn.conv2d(self.input, W_conv0, strides=[1, 2, 2, 1], padding='SAME', name='Teacher_conv0')
        tmp = tf.nn.bias_add(tmp, b_conv0)
        tmp = tf.nn.relu(tmp)

        W_conv1 = tf.get_variable('Teacher/W_conv1', shape=[5, 5, 24, 36],
                                  initializer=initializer)
        b_conv1 = tf.get_variable('Teacher/b_conv1', shape=[36], initializer=initializer)
        tmp = tf.nn.conv2d(tmp, W_conv1, strides=[1, 2, 2, 1], padding='SAME', name='Teacher_conv1')
        tmp = tf.nn.bias_add(tmp, b_conv1)
        tmp = tf.nn.relu(tmp)

        W_conv2 = tf.get_variable('Teacher/W_conv2', shape=[5, 5, 36, 48],
                                  initializer=initializer)
        b_conv2 = tf.get_variable('Teacher/b_conv2', shape=[48], initializer=initializer)
        tmp = tf.nn.conv2d(tmp, W_conv2, strides=[1, 2, 2, 1], padding='SAME', name='Teacher_conv2')
        tmp = tf.nn.bias_add(tmp, b_conv2)
        tmp = tf.nn.relu(tmp)

        W_conv3 = tf.get_variable('Teacher/W_conv3', shape=[3, 3, 48, 64],
                                  initializer=initializer)
        b_conv3 = tf.get_variable('Teacher/b_conv3', shape=[64], initializer=initializer)
        tmp = tf.nn.conv2d(tmp, W_conv3, strides=[1, 1, 1, 1], padding='SAME', name='Teacher_conv3')
        tmp = tf.nn.bias_add(tmp, b_conv3)
        tmp = tf.nn.relu(tmp)

        W_conv4 = tf.get_variable('Teacher/W_conv4', shape=[3, 3, 64, 64],
                                  initializer=initializer)
        b_conv4 = tf.get_variable('Teacher/b_conv4', shape=[64], initializer=initializer)
        tmp = tf.nn.conv2d(tmp, W_conv4, strides=[1, 1, 1, 1], padding='SAME', name='Teacher_conv4')
        tmp = tf.nn.bias_add(tmp, b_conv4)
        tmp = tf.nn.relu(tmp)

        # Fully connected layers
        tmp = tf.contrib.layers.flatten(tmp)
        W_fc0 = tf.get_variable('Teacher/W_fc0', shape=[14400, 1164],
                                initializer=initializer)
        b_fc0 = tf.get_variable('Teacher/b_fc0', shape=[1164], initializer=initializer)
        tmp = tf.nn.relu(tf.matmul(tmp, W_fc0) + b_fc0)
        self.hint = tmp

        W_fc1 = tf.get_variable('Teacher/W_fc1', shape=[1164, 100], initializer=initializer)
        b_fc1 = tf.get_variable('Teacher/b_fc1', shape=[100], initializer=initializer)
        tmp = tf.nn.relu(tf.matmul(tmp, W_fc1) + b_fc1)

        W_fc2 = tf.get_variable('Teacher/W_fc2', shape=[100, 50], initializer=initializer)
        b_fc2 = tf.get_variable('Teacher/b_fc2', shape=[50], initializer=initializer)
        tmp = tf.nn.relu(tf.matmul(tmp, W_fc2) + b_fc2)

        W_fc3 = tf.get_variable('Teacher/W_fc3', shape=[50, 10], initializer=initializer)
        b_fc3 = tf.get_variable('Teacher/b_fc3', shape=[10], initializer=initializer)
        tmp = tf.nn.relu(tf.matmul(tmp, W_fc3) + b_fc3)

        W_fc4 = tf.get_variable('Teacher/W_fc4', shape=[10, 1], initializer=initializer)
        b_fc4 = tf.get_variable('Teacher/b_fc4', shape=[1], initializer=initializer)
        self.steering = tf.matmul(tmp, W_fc4) + b_fc4

        self.loss = tf.losses.mean_squared_error(self.steering_ref, self.steering)

        all_vars = tf.trainable_variables()
        teacher_vars = [var for var in all_vars if 'Teacher' in var.name]
        self.optimizer = tf.train.AdamOptimizer(1e-4).minimize(self.loss, var_list=teacher_vars)


class Teacher_depthwise(object):
    def __init__(self):
        self.input = tf.placeholder(tf.float32, shape=(None, 66, 200, 3), name='teacher_input')
        self.steering_ref = tf.placeholder(tf.float32, shape=(None, 1), name='teacher_steering')

        # Convolutional layers
        # W_conv0 = tf.get_variable('Teacher/W_conv0', shape=[5, 5, 3, 24],
        #                           initializer=initializer)
        # b_conv0 = tf.get_variable('Teacher/b_conv0', shape=[24], initializer=initializer)
        # tmp = tf.nn.separable_conv2d(self.input, W_conv0, strides=[1, 2, 2, 1], padding='SAME', name='Teacher_conv0')
        # tmp = tf.nn.bias_add(tmp, b_conv0)
        # tmp = tf.nn.relu(tmp)

        tmp = tf.layers.separable_conv2d(self.input, filters=24, kernel_size=[5, 5], strides=[2, 2], padding='SAME',
                                         activation=tf.nn.relu, name='Teacher_conv0')

        # W_conv1 = tf.get_variable('Teacher/W_conv1', shape=[5, 5, 24, 36],
        #                           initializer=initializer)
        # b_conv1 = tf.get_variable('Teacher/b_conv1', shape=[36], initializer=initializer)
        # tmp = tf.nn.separable_conv2d(tmp, W_conv1, strides=[1, 2, 2, 1], padding='SAME', name='Teacher_conv1')
        # tmp = tf.nn.bias_add(tmp, b_conv1)
        # tmp = tf.nn.relu(tmp)

        tmp = tf.layers.separable_conv2d(tmp, filters=36, kernel_size=[5, 5], strides=[2, 2], padding='SAME',
                                        activation=tf.nn.relu, name='Teacher_conv1')

        # W_conv2 = tf.get_variable('Teacher/W_conv2', shape=[5, 5, 36, 48],
        #                           initializer=initializer)
        # b_conv2 = tf.get_variable('Teacher/b_conv2', shape=[48], initializer=initializer)
        # tmp = tf.nn.separable_conv2d(tmp, W_conv2, strides=[1, 2, 2, 1], padding='SAME', name='Teacher_conv2')
        # tmp = tf.nn.bias_add(tmp, b_conv2)
        # tmp = tf.nn.relu(tmp)

        tmp = tf.layers.separable_conv2d(tmp, filters=48, kernel_size=[5, 5], strides=[2, 2], padding='SAME',
                                         activation=tf.nn.relu, name='Teacher_conv2')

        # W_conv3 = tf.get_variable('Teacher/W_conv3', shape=[3, 3, 48, 64],
        #                           initializer=initializer)
        # b_conv3 = tf.get_variable('Teacher/b_conv3', shape=[64], initializer=initializer)
        # tmp = tf.nn.separable_conv2d(tmp, W_conv3, strides=[1, 1, 1, 1], padding='SAME', name='Teacher_conv3')
        # tmp = tf.nn.bias_add(tmp, b_conv3)
        # tmp = tf.nn.relu(tmp)

        tmp = tf.layers.separable_conv2d(tmp, filters=64, kernel_size=[3, 3], strides=[1, 1], padding='SAME',
                                         activation=tf.nn.relu, name='Teacher_conv3')

        # W_conv4 = tf.get_variable('Teacher/W_conv4', shape=[3, 3, 64, 64],
        #                           initializer=initializer)
        # b_conv4 = tf.get_variable('Teacher/b_conv4', shape=[64], initializer=initializer)
        # tmp = tf.nn.separable_conv2d(tmp, W_conv4, strides=[1, 1, 1, 1], padding='SAME', name='Teacher_conv4')
        # tmp = tf.nn.bias_add(tmp, b_conv4)
        # tmp = tf.nn.relu(tmp)

        tmp = tf.layers.separable_conv2d(tmp, filters=64, kernel_size=[3, 3], strides=[1, 1], padding='SAME',
                                         activation=tf.nn.relu, name='Teacher_conv4')

        # Fully connected layers
        tmp = tf.contrib.layers.flatten(tmp)
        W_fc0 = tf.get_variable('Teacher/W_fc0', shape=[14400, 1164],
                                initializer=initializer)
        b_fc0 = tf.get_variable('Teacher/b_fc0', shape=[1164], initializer=initializer)
        tmp = tf.nn.relu(tf.matmul(tmp, W_fc0) + b_fc0)
        self.hint = tmp

        W_fc1 = tf.get_variable('Teacher/W_fc1', shape=[1164, 100], initializer=initializer)
        b_fc1 = tf.get_variable('Teacher/b_fc1', shape=[100], initializer=initializer)
        tmp = tf.nn.relu(tf.matmul(tmp, W_fc1) + b_fc1)

        W_fc2 = tf.get_variable('Teacher/W_fc2', shape=[100, 50], initializer=initializer)
        b_fc2 = tf.get_variable('Teacher/b_fc2', shape=[50], initializer=initializer)
        tmp = tf.nn.relu(tf.matmul(tmp, W_fc2) + b_fc2)

        W_fc3 = tf.get_variable('Teacher/W_fc3', shape=[50, 10], initializer=initializer)
        b_fc3 = tf.get_variable('Teacher/b_fc3', shape=[10], initializer=initializer)
        tmp = tf.nn.relu(tf.matmul(tmp, W_fc3) + b_fc3)

        W_fc4 = tf.get_variable('Teacher/W_fc4', shape=[10, 1], initializer=initializer)
        b_fc4 = tf.get_variable('Teacher/b_fc4', shape=[1], initializer=initializer)
        self.steering = tf.matmul(tmp, W_fc4) + b_fc4

        self.loss = tf.losses.mean_squared_error(self.steering_ref, self.steering)

        all_vars = tf.trainable_variables()
        teacher_vars = [var for var in all_vars if 'Teacher' in var.name]
        self.optimizer = tf.train.AdamOptimizer(1e-4).minimize(self.loss, var_list=teacher_vars)


class Student(object):
    def __init__(self, teacher, width=1, distillation=False, temperature=1, use_hint=False):
        self.input = tf.placeholder(tf.float32, shape=(None, 66, 200, 3), name='student_input')
        self.steering_ref = tf.placeholder(tf.float32, shape=(None, 1), name='student_steering')

        # Convolutional layers
        W_conv0 = tf.get_variable('Student/W_conv0', shape=[5, 5, 3, int(24 * width)],
                                  initializer=initializer)
        b_conv0 = tf.get_variable('Student/b_conv0', shape=[int(24 * width)],
                                  initializer=initializer)
        tmp = tf.nn.conv2d(self.input, W_conv0, strides=[1, 2, 2, 1], padding='SAME', name='Student_conv0')
        tmp = tf.nn.bias_add(tmp, b_conv0)
        tmp = tf.nn.relu(tmp)

        W_conv1 = tf.get_variable('Student/W_conv1', shape=[5, 5, int(24 * width), int(36 * width)],
                                  initializer=initializer)
        b_conv1 = tf.get_variable('Student/b_conv1', shape=[int(36 * width)],
                                  initializer=initializer)
        tmp = tf.nn.conv2d(tmp, W_conv1, strides=[1, 2, 2, 1], padding='SAME', name='Student_conv1')
        tmp = tf.nn.bias_add(tmp, b_conv1)
        tmp = tf.nn.relu(tmp)

        W_conv2 = tf.get_variable('Student/W_conv2', shape=[5, 5, int(36 * width), int(48 * width)],
                                  initializer=initializer)
        b_conv2 = tf.get_variable('Student/b_conv2', shape=[int(48 * width)],
                                  initializer=initializer)
        tmp = tf.nn.conv2d(tmp, W_conv2, strides=[1, 2, 2, 1], padding='SAME', name='Student_conv2')
        tmp = tf.nn.bias_add(tmp, b_conv2)
        tmp = tf.nn.relu(tmp)

        W_conv3 = tf.get_variable('Student/W_conv3', shape=[3, 3, int(48 * width), int(64 * width)],
                                  initializer=initializer)
        b_conv3 = tf.get_variable('Student/b_conv3', shape=[int(64 * width)],
                                  initializer=initializer)
        tmp = tf.nn.conv2d(tmp, W_conv3, strides=[1, 1, 1, 1], padding='SAME', name='Student_conv3')
        tmp = tf.nn.bias_add(tmp, b_conv3)
        tmp = tf.nn.relu(tmp)

        W_conv4 = tf.get_variable('Student/W_conv4', shape=[3, 3, int(64 * width), int(64 * width)],
                                  initializer=initializer)
        b_conv4 = tf.get_variable('Student/b_conv4', shape=[int(64 * width)],
                                  initializer=initializer)
        tmp = tf.nn.conv2d(tmp, W_conv4, strides=[1, 1, 1, 1], padding='SAME', name='Student_conv4')
        tmp = tf.nn.bias_add(tmp, b_conv4)
        tmp = tf.nn.relu(tmp)

        # Fully connected layers
        tmp = tf.contrib.layers.flatten(tmp)
        W_fc0 = tf.get_variable('Student/W_fc0', shape=[int(14400 * width), 1164],
                                initializer=initializer)
        b_fc0 = tf.get_variable('Student/b_fc0', shape=[1164], initializer=initializer)
        tmp = tf.nn.relu(tf.matmul(tmp, tf.contrib.model_pruning.apply_mask(W_fc0)) + b_fc0)
        self.hint = tmp

        W_fc1 = tf.get_variable('Student/W_fc1', shape=[1164, 100], initializer=initializer)
        b_fc1 = tf.get_variable('Student/b_fc1', shape=[100], initializer=initializer)
        tmp = tf.nn.relu(tf.matmul(tmp, tf.contrib.model_pruning.apply_mask(W_fc1, scope='fc1')) + b_fc1)

        W_fc2 = tf.get_variable('Student/W_fc2', shape=[100, 50], initializer=initializer)
        b_fc2 = tf.get_variable('Student/b_fc2', shape=[50], initializer=initializer)
        tmp = tf.nn.relu(tf.matmul(tmp, tf.contrib.model_pruning.apply_mask(W_fc2, scope='fc2')) + b_fc2)

        W_fc3 = tf.get_variable('Student/W_fc3', shape=[50, 10], initializer=initializer)
        b_fc3 = tf.get_variable('Student/b_fc3', shape=[10], initializer=initializer)
        tmp = tf.nn.relu(tf.matmul(tmp, tf.contrib.model_pruning.apply_mask(W_fc3, scope='fc3')) + b_fc3)

        W_fc4 = tf.get_variable('Student/W_fc4', shape=[10, 1], initializer=initializer)
        b_fc4 = tf.get_variable('Student/b_fc4', shape=[1], initializer=initializer)
        self.steering = tf.matmul(tmp, W_fc4) + b_fc4

        if distillation == False:
            self.loss = tf.losses.mean_squared_error(self.steering_ref, self.steering)
        else:
            if use_hint == False:
                self.loss = tf.losses.mean_squared_error(teacher.steering / temperature, self.steering / temperature)
            else:
                self.loss = tf.losses.mean_squared_error(teacher.steering / temperature, self.steering / temperature)
                + 1e-4 * tf.losses.mean_squared_error(teacher.hint, self.hint)

        all_vars = tf.trainable_variables()
        student_vars = [var for var in all_vars if 'Student' in var.name]
        self.optimizer = tf.train.AdamOptimizer(1e-4).minimize(self.loss, var_list=student_vars)
