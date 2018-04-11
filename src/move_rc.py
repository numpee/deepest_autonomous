#!/usr/bin/env python

import rospy

from std_msgs.msg import Int16

import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Controlling the robot:

a       d

w: go forward
a, d: turn left/right
s: stop
CTRL-C to quit
"""

moveBindings = {
        'a':(-5),
        'd':(5),
        }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 1


def vels(direction):
    if direction=='w':
        return "currently:\tdirection: forward"
    elif direction=='a':
        return "currently:\tdirection: turn left"
    elif direction=='s':
        return "currently:\tdirection: stop"
    elif direction=='d':
        return "currently:\tdirection: turn right"
    elif direction=='x':
        return "currently:\tdirection: back"
    else:
        return "currently:\tdirection: stop"


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('deepest_teleop')
    pub = rospy.Publisher('/cmd_vel', Int16, queue_size=5)

    steer = 90
    try:
        print msg
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                steer += moveBindings[key]
                print(steer)
            else:
                if (key == '\x03'):
                    break

            pub.publish(steer)

    except Exception as e:
        print e

    finally:
        pub.publish(steer)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
