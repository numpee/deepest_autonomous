#!/usr/bin/env python

import rospy

from std_msgs.msg import Int16

import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Controlling the robot:
    w
a   s   d
    x

w: go forward
a, d: turn left/right
s: stop
CTRL-C to quit
"""

moveBindings = {
        'w':(1),
        'a':(2),
        's':(3),
        'd':(4),
        'x':(5),
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
    
    rospy.init_node('moya_teleop')
    pub = rospy.Publisher('/cmd_vel', Int16, queue_size=5)

    robot_direction=3
    try:
        print msg
        print vels(speed)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                robot_direction = moveBindings[key]
                print(vels(key))
            elif key == ' ' or key == 'k' :
                robot_direction = 3
            else:
                if (key == '\x03'):
                    break

            pub.publish(robot_direction)

    except Exception as e:
        print e

    finally:
        pub.publish(robot_direction)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
